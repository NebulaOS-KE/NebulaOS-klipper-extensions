# nebulaos_plr_journal.py - EEPROM journal codec for NebulaOS power-loss
# recovery (Phase 1.9B).
#
# Pure stdlib, zero Klipper dependencies - deliberately, so this module is
# usable both as nebulaos_power_loss_recovery.py's own EEPROM backend and as
# a standalone system tool (plr_tombstone.py, invoked from the NebulaOS ->
# stock switch entry point, where Klippy may not even be running). Every
# function here takes a plain file-like object (anything with
# seek()/read()/write()/flush()) rather than a path, so unit tests can drive
# it against an in-memory fake (see test_nebulaos_plr_journal.py) without
# touching any real filesystem or I2C device.
#
# Physical layout - BL24C16F, 2048 bytes total, exposed by the Linux 6.6
# in-tree "atmel,24c16" at24/nvmem driver (NOT [bl24c16f], which owned this
# chip directly over i2c_mcu/i2c-chardev in Phase 1.9A and is retired from
# production use in Phase 1.9B - see machine.cfg's own history). The
# hardware's native write-page size is 16 bytes, which this module's own
# record size matches exactly - every journal write is a single, atomic,
# page-aligned hardware write, never split across a page boundary.
#
#   page 0            (bytes 0..15)     - RESERVED for stock Creality PLR.
#                                          byte 0 = stock checkpoint slot
#                                          pointer, byte 1 = stock enabled/
#                                          valid state (0xff = disabled).
#                                          NebulaOS NEVER reads or writes
#                                          this page - see STOCK_PAGE below.
#   pages 1..127      (bytes 16..2032)  - NebulaOS journal, one 16-byte
#                                          record per page, committed by
#                                          advancing around the ring. No
#                                          mutable header page anywhere -
#                                          "the current state" is always
#                                          derived by scanning every page,
#                                          never cached in a fixed location.
#
# Record format (little-endian, 16 bytes total):
#   +0   2 bytes   magic b"NP"
#   +2   u8        schema_version (1)
#   +3   u8        flags (FLAG_VALID | FLAG_TOMBSTONE)
#   +4   u32       generation
#   +8   u32       file_position
#   +12  u32       CRC32(bytes 0..11)
#
# Generation is compared with wrap-safe u32 serial-number arithmetic (RFC
# 1982) throughout, and pages are reused circularly - if the newest write to
# a given page is torn (e.g. genuine power loss mid-write), decode_record()
# simply rejects it on bad CRC and scan_journal() falls back to the
# next-highest-generation valid record on a different page, which is
# necessarily the previous good checkpoint (see commit_checkpoint()).
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import struct
import zlib

PAGE_SIZE = 16
EEPROM_TOTAL_SIZE = 2048

STOCK_PAGE = 0
JOURNAL_FIRST_PAGE = 1
JOURNAL_LAST_PAGE = 127  # inclusive
JOURNAL_PAGE_COUNT = JOURNAL_LAST_PAGE - JOURNAL_FIRST_PAGE + 1  # 127

MAGIC = b"NP"
SCHEMA_VERSION = 1

FLAG_VALID = 0x01
FLAG_TOMBSTONE = 0x02

# magic(2s) + schema_version(B) + flags(B) + generation(I) + file_position(I)
_HEADER_STRUCT = struct.Struct('<2sBBII')
_CRC_STRUCT = struct.Struct('<I')

assert _HEADER_STRUCT.size + _CRC_STRUCT.size == PAGE_SIZE
assert JOURNAL_LAST_PAGE * PAGE_SIZE + PAGE_SIZE == EEPROM_TOTAL_SIZE


class Record(object):
    __slots__ = ('page', 'schema_version', 'flags', 'generation', 'file_position')

    def __init__(self, page, schema_version, flags, generation, file_position):
        self.page = page
        self.schema_version = schema_version
        self.flags = flags
        self.generation = generation
        self.file_position = file_position

    @property
    def is_tombstone(self):
        return bool(self.flags & FLAG_TOMBSTONE)

    @property
    def is_valid_flag(self):
        return bool(self.flags & FLAG_VALID)

    def __repr__(self):
        return ("Record(page=%r, flags=0x%02x, generation=%d, file_position=%d)"
                % (self.page, self.flags, self.generation, self.file_position))

    def __eq__(self, other):
        if not isinstance(other, Record):
            return NotImplemented
        return (self.page == other.page and self.schema_version == other.schema_version
                and self.flags == other.flags and self.generation == other.generation
                and self.file_position == other.file_position)


def encode_record(schema_version, flags, generation, file_position):
    header = _HEADER_STRUCT.pack(MAGIC, schema_version, flags,
                                  generation & 0xFFFFFFFF,
                                  file_position & 0xFFFFFFFF)
    crc = zlib.crc32(header) & 0xFFFFFFFF
    data = header + _CRC_STRUCT.pack(crc)
    assert len(data) == PAGE_SIZE
    return data


def decode_record(data, page=None):
    """Returns a Record, or None if the page is short, or its magic/
    version/CRC don't check out. This is the ONLY acceptance gate - any
    structurally-invalid page (including a torn write) is simply ignored
    by the caller, never raised as an error."""
    if data is None or len(data) != PAGE_SIZE:
        return None
    header = data[:_HEADER_STRUCT.size]
    (crc_stored,) = _CRC_STRUCT.unpack(data[_HEADER_STRUCT.size:])
    if (zlib.crc32(header) & 0xFFFFFFFF) != crc_stored:
        return None
    magic, schema_version, flags, generation, file_position = \
        _HEADER_STRUCT.unpack(header)
    if magic != MAGIC:
        return None
    if schema_version != SCHEMA_VERSION:
        return None
    return Record(page, schema_version, flags, generation, file_position)


def generation_is_newer(a, b):
    """Wrap-safe u32 sequence comparison (RFC 1982 serial number
    arithmetic). True if generation `a` is strictly newer than `b`."""
    a &= 0xFFFFFFFF
    b &= 0xFFFFFFFF
    diff = (a - b) & 0xFFFFFFFF
    return diff != 0 and diff < 0x80000000


def read_page(fileobj, page):
    fileobj.seek(page * PAGE_SIZE)
    data = fileobj.read(PAGE_SIZE)
    if data is None or len(data) != PAGE_SIZE:
        return None
    return data


def write_page_verified(fileobj, page, data):
    """Writes one page, then reads it back and verifies byte-for-byte -
    every checkpoint and tombstone commit requires this; a write is never
    trusted just because the OS call returned success (mission step 7:
    "read EEPROM record back and verify bytes/CRC")."""
    assert len(data) == PAGE_SIZE
    fileobj.seek(page * PAGE_SIZE)
    fileobj.write(data)
    fileobj.flush()
    readback = read_page(fileobj, page)
    return readback == data


def scan_journal(fileobj):
    """Scans pages JOURNAL_FIRST_PAGE..JOURNAL_LAST_PAGE and returns the
    Record with the newest generation among structurally-valid records, or
    None if no valid record exists anywhere in the journal (fresh/blank
    EEPROM, or every page happens to be corrupt)."""
    newest = None
    for page in range(JOURNAL_FIRST_PAGE, JOURNAL_LAST_PAGE + 1):
        record = decode_record(read_page(fileobj, page), page=page)
        if record is None:
            continue
        if newest is None or generation_is_newer(record.generation, newest.generation):
            newest = record
    return newest


def newer_schema_present(fileobj):
    """G100 R1 (Run 21): True if a page holds a structurally valid record
    (magic + CRC) of a NEWER schema than this code reads. decode_record()
    ignores such pages like torn ones; this only lets the caller say so
    after a rollback to an older NebulaOS, instead of silently showing
    "no recovery available"."""
    for page in range(JOURNAL_FIRST_PAGE, JOURNAL_LAST_PAGE + 1):
        data = read_page(fileobj, page)
        if data is None or len(data) != PAGE_SIZE:
            continue
        header = data[:_HEADER_STRUCT.size]
        (crc_stored,) = _CRC_STRUCT.unpack(data[_HEADER_STRUCT.size:])
        if (zlib.crc32(header) & 0xFFFFFFFF) != crc_stored:
            continue
        magic, schema_version = _HEADER_STRUCT.unpack(header)[:2]
        if magic == MAGIC and schema_version > SCHEMA_VERSION:
            return True
    return False


def next_commit_target(current):
    """Given the current newest Record (or None if the journal is empty),
    returns (next_page, next_generation) for the next commit - advancing
    around the pages 1..127 ring, generation always strictly increasing
    (wrap-safe). No mutable header is consulted or updated; the next
    target is entirely re-derived from `current` (itself the result of a
    fresh scan_journal() call) every time."""
    if current is None:
        return JOURNAL_FIRST_PAGE, 1
    next_page = current.page + 1
    if next_page > JOURNAL_LAST_PAGE:
        next_page = JOURNAL_FIRST_PAGE
    next_generation = (current.generation + 1) & 0xFFFFFFFF
    if next_generation == 0:
        # Skip generation 0 - keeping it permanently unused avoids any
        # ambiguity with "no record" sentinels in callers that might
        # otherwise treat 0 as falsy/unset.
        next_generation = 1
    return next_page, next_generation


def read_recovery_state(fileobj):
    """Implements the mission's read algorithm exactly: scan all journal
    pages, reject bad magic/version/CRC, choose the newest generation; if
    the newest valid record is a tombstone, there is no recovery
    available. Returns the newest non-tombstoned Record to recover from,
    or None (either the journal is empty, or the latest state is a
    tombstone)."""
    newest = scan_journal(fileobj)
    if newest is None or newest.is_tombstone:
        return None
    return newest


def commit_checkpoint(fileobj, file_position):
    """Writes a new VALID checkpoint record advancing the ring, verifies
    the write by reading it back. Returns the committed Record. Raises
    IOError if the readback verification fails (mission step 7) - the
    caller must treat that as a failed checkpoint, not a partial one."""
    current = scan_journal(fileobj)
    page, generation = next_commit_target(current)
    data = encode_record(SCHEMA_VERSION, FLAG_VALID, generation, file_position)
    if not write_page_verified(fileobj, page, data):
        raise IOError("PLR journal commit verification failed at page %d" % page)
    return decode_record(data, page=page)


def commit_tombstone(fileobj):
    """Writes a new TOMBSTONE record advancing the ring (so it becomes the
    newest record and read_recovery_state() correctly reports "no
    recovery"), verifies the write. Safe to call on an empty journal (a
    tombstone with no prior checkpoint is a harmless no-op state - e.g. a
    fresh device that switches to stock before ever printing). Returns the
    committed Record. Raises IOError if readback verification fails."""
    current = scan_journal(fileobj)
    page, generation = next_commit_target(current)
    data = encode_record(SCHEMA_VERSION, FLAG_TOMBSTONE, generation, 0)
    if not write_page_verified(fileobj, page, data):
        raise IOError("PLR journal tombstone verification failed at page %d" % page)
    return decode_record(data, page=page)


def sidecar_parity(generation):
    """Generation parity selects the sidecar file - even generations use
    slot 0 ("state-a.json"), odd generations use slot 1 ("state-b.json").
    A torn write to the OTHER (non-selected) file can never corrupt the
    generation the EEPROM record actually points to."""
    return generation & 1
