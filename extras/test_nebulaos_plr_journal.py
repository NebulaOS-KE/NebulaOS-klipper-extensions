# nebulaos_plr_journal tests - pure codec, zero hardware/filesystem/Klipper
# dependency. The fake EEPROM below is a plain io.BytesIO pre-sized to the
# real chip's 2048 bytes, exercised through exactly the same file-like
# seek()/read()/write()/flush() interface the real nvmem sysfs path
# provides - so every test here genuinely exercises the same code path
# that will run against real hardware, only the storage backing differs.
#
# Run from klippy/: python3 -m unittest extras.test_nebulaos_plr_journal -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import io
import struct
import unittest

from . import nebulaos_plr_journal as plr


def fake_eeprom(fill=0xFF):
    return io.BytesIO(bytes([fill]) * plr.EEPROM_TOTAL_SIZE)


class RecordEncodeDecodeTests(unittest.TestCase):
    def test_round_trip(self):
        data = plr.encode_record(plr.SCHEMA_VERSION, plr.FLAG_VALID, 42, 123456)
        self.assertEqual(len(data), plr.PAGE_SIZE)
        record = plr.decode_record(data, page=5)
        self.assertIsNotNone(record)
        self.assertEqual(record.page, 5)
        self.assertEqual(record.schema_version, plr.SCHEMA_VERSION)
        self.assertEqual(record.flags, plr.FLAG_VALID)
        self.assertEqual(record.generation, 42)
        self.assertEqual(record.file_position, 123456)
        self.assertTrue(record.is_valid_flag)
        self.assertFalse(record.is_tombstone)

    def test_tombstone_flag(self):
        data = plr.encode_record(plr.SCHEMA_VERSION, plr.FLAG_TOMBSTONE, 1, 0)
        record = plr.decode_record(data)
        self.assertTrue(record.is_tombstone)

    def test_wrong_length_rejected(self):
        self.assertIsNone(plr.decode_record(b"short"))
        self.assertIsNone(plr.decode_record(b"x" * (plr.PAGE_SIZE + 1)))

    def test_none_rejected(self):
        self.assertIsNone(plr.decode_record(None))


class CrcRejectionTests(unittest.TestCase):
    def test_flipped_bit_in_header_rejected(self):
        data = bytearray(plr.encode_record(plr.SCHEMA_VERSION, plr.FLAG_VALID, 7, 99))
        data[4] ^= 0x01  # flip a bit inside the generation field
        self.assertIsNone(plr.decode_record(bytes(data)))

    def test_flipped_bit_in_crc_itself_rejected(self):
        data = bytearray(plr.encode_record(plr.SCHEMA_VERSION, plr.FLAG_VALID, 7, 99))
        data[-1] ^= 0x01
        self.assertIsNone(plr.decode_record(bytes(data)))

    def test_bad_magic_rejected(self):
        # Hand-craft a page with a correct CRC over a bad magic, proving
        # decode_record() checks magic independently of CRC passing.
        header = struct.pack('<2sBBII', b'XX', plr.SCHEMA_VERSION,
                              plr.FLAG_VALID, 1, 0)
        import zlib
        crc = zlib.crc32(header) & 0xFFFFFFFF
        data = header + struct.pack('<I', crc)
        self.assertIsNone(plr.decode_record(data))

    def test_bad_schema_version_rejected(self):
        header = struct.pack('<2sBBII', plr.MAGIC, plr.SCHEMA_VERSION + 1,
                              plr.FLAG_VALID, 1, 0)
        import zlib
        crc = zlib.crc32(header) & 0xFFFFFFFF
        data = header + struct.pack('<I', crc)
        self.assertIsNone(plr.decode_record(data))

    def test_blank_erased_eeprom_page_rejected(self):
        # A freshly-erased/never-written EEPROM page reads back as all
        # 0xff - must not be mistaken for a valid record.
        self.assertIsNone(plr.decode_record(b'\xff' * plr.PAGE_SIZE))

    def test_all_zero_page_rejected(self):
        self.assertIsNone(plr.decode_record(b'\x00' * plr.PAGE_SIZE))


class GenerationOrderingWrapTests(unittest.TestCase):
    def test_simple_newer(self):
        self.assertTrue(plr.generation_is_newer(5, 3))
        self.assertFalse(plr.generation_is_newer(3, 5))

    def test_equal_is_not_newer(self):
        self.assertFalse(plr.generation_is_newer(5, 5))

    def test_wraparound_newer(self):
        # 2 is newer than 0xFFFFFFFE (wrapped past the u32 boundary).
        self.assertTrue(plr.generation_is_newer(2, 0xFFFFFFFE))
        self.assertFalse(plr.generation_is_newer(0xFFFFFFFE, 2))

    def test_exact_half_boundary_is_ambiguous_but_defined(self):
        # RFC 1982: a difference of exactly 2**31 is defined as
        # "not newer" in this implementation (diff < 0x80000000 is a
        # strict less-than) - just confirm it doesn't crash and picks a
        # consistent, stable side.
        a = 0
        b = 0x80000000
        self.assertFalse(plr.generation_is_newer(a, b))


class JournalRotationTests(unittest.TestCase):
    def test_empty_journal_scan_is_none(self):
        eeprom = fake_eeprom()
        self.assertIsNone(plr.scan_journal(eeprom))
        self.assertIsNone(plr.read_recovery_state(eeprom))

    def test_first_commit_targets_page_1_generation_1(self):
        eeprom = fake_eeprom()
        record = plr.commit_checkpoint(eeprom, file_position=1000)
        self.assertEqual(record.page, plr.JOURNAL_FIRST_PAGE)
        self.assertEqual(record.generation, 1)
        self.assertEqual(record.file_position, 1000)

    def test_successive_commits_advance_page_and_generation(self):
        eeprom = fake_eeprom()
        r1 = plr.commit_checkpoint(eeprom, 100)
        r2 = plr.commit_checkpoint(eeprom, 200)
        r3 = plr.commit_checkpoint(eeprom, 300)
        self.assertEqual([r1.page, r2.page, r3.page], [1, 2, 3])
        self.assertEqual([r1.generation, r2.generation, r3.generation], [1, 2, 3])

    def test_ring_wraps_after_last_page(self):
        eeprom = fake_eeprom()
        current = None
        for _ in range(plr.JOURNAL_PAGE_COUNT):
            current = plr.commit_checkpoint(eeprom, current.file_position + 1
                                             if current else 0)
        self.assertEqual(current.page, plr.JOURNAL_LAST_PAGE)
        wrapped = plr.commit_checkpoint(eeprom, 9999)
        self.assertEqual(wrapped.page, plr.JOURNAL_FIRST_PAGE)
        self.assertEqual(wrapped.generation, plr.JOURNAL_PAGE_COUNT + 1)

    def test_newer_schema_present(self):
        # G100 R1: a valid page of a newer schema is ignored by the scan
        # but reported; an older/equal schema or a torn page is not.
        eeprom = fake_eeprom()
        self.assertFalse(plr.newer_schema_present(eeprom))
        plr.commit_checkpoint(eeprom, 100)
        self.assertFalse(plr.newer_schema_present(eeprom))
        plr.write_page_verified(eeprom, 5, plr.encode_record(plr.SCHEMA_VERSION + 1,
                                                    plr.FLAG_VALID, 9, 500))
        self.assertTrue(plr.newer_schema_present(eeprom))
        self.assertEqual(plr.scan_journal(eeprom).generation, 1)
        torn = bytearray(plr.encode_record(plr.SCHEMA_VERSION + 1, plr.FLAG_VALID, 9, 500))
        torn[4] ^= 1
        plr.write_page_verified(eeprom, 5, bytes(torn))
        self.assertFalse(plr.newer_schema_present(eeprom))

    def test_scan_after_wrap_finds_newest_not_oldest(self):
        eeprom = fake_eeprom()
        current = None
        for i in range(plr.JOURNAL_PAGE_COUNT + 3):
            current = plr.commit_checkpoint(eeprom, i)
        newest = plr.scan_journal(eeprom)
        self.assertEqual(newest.generation, current.generation)
        self.assertEqual(newest.file_position, current.file_position)

    def test_read_recovery_state_returns_latest_checkpoint(self):
        eeprom = fake_eeprom()
        plr.commit_checkpoint(eeprom, 10)
        plr.commit_checkpoint(eeprom, 20)
        state = plr.read_recovery_state(eeprom)
        self.assertEqual(state.file_position, 20)


class TornRecordSurvivalTests(unittest.TestCase):
    def test_torn_newest_record_previous_valid_survives(self):
        eeprom = fake_eeprom()
        good = plr.commit_checkpoint(eeprom, 111)
        next_page, next_gen = plr.next_commit_target(good)
        # Simulate a torn write: write a record but corrupt one CRC byte
        # afterward, as a partial/interrupted physical write would leave
        # behind (structurally present, but not verifiable).
        torn = bytearray(plr.encode_record(plr.SCHEMA_VERSION, plr.FLAG_VALID,
                                            next_gen, 222))
        torn[-1] ^= 0xFF
        eeprom.seek(next_page * plr.PAGE_SIZE)
        eeprom.write(bytes(torn))
        eeprom.flush()

        recovered = plr.read_recovery_state(eeprom)
        self.assertIsNotNone(recovered)
        self.assertEqual(recovered.file_position, 111)
        self.assertEqual(recovered.generation, good.generation)

    def test_write_page_verified_detects_readback_mismatch(self):
        class LyingFile(object):
            """A fake device that silently writes the wrong bytes -
            models a hardware fault write_page_verified() must catch."""
            def __init__(self):
                self.buf = bytearray(plr.EEPROM_TOTAL_SIZE)
                self.pos = 0

            def seek(self, pos):
                self.pos = pos

            def read(self, n):
                data = bytes(self.buf[self.pos:self.pos + n])
                self.pos += len(data)
                return data

            def write(self, data):
                # Corrupt one byte of whatever gets written.
                corrupted = bytearray(data)
                corrupted[0] ^= 0xFF
                self.buf[self.pos:self.pos + len(corrupted)] = corrupted
                self.pos += len(corrupted)

            def flush(self):
                pass

        lying = LyingFile()
        data = plr.encode_record(plr.SCHEMA_VERSION, plr.FLAG_VALID, 1, 0)
        self.assertFalse(plr.write_page_verified(lying, 1, data))


class TombstoneTests(unittest.TestCase):
    def test_tombstone_after_checkpoint_means_no_recovery(self):
        eeprom = fake_eeprom()
        plr.commit_checkpoint(eeprom, 500)
        plr.commit_tombstone(eeprom)
        self.assertIsNone(plr.read_recovery_state(eeprom))

    def test_tombstone_on_empty_journal_is_harmless(self):
        eeprom = fake_eeprom()
        record = plr.commit_tombstone(eeprom)
        self.assertTrue(record.is_tombstone)
        self.assertIsNone(plr.read_recovery_state(eeprom))

    def test_checkpoint_after_tombstone_recovers_again(self):
        eeprom = fake_eeprom()
        plr.commit_checkpoint(eeprom, 1)
        plr.commit_tombstone(eeprom)
        plr.commit_checkpoint(eeprom, 999)
        state = plr.read_recovery_state(eeprom)
        self.assertEqual(state.file_position, 999)

    def test_tombstone_generation_advances_ring_same_as_checkpoint(self):
        eeprom = fake_eeprom()
        c = plr.commit_checkpoint(eeprom, 1)
        t = plr.commit_tombstone(eeprom)
        self.assertEqual(t.page, c.page + 1)
        self.assertEqual(t.generation, c.generation + 1)


class StockPageNeverTouchedTests(unittest.TestCase):
    def test_stock_page_bytes_untouched_by_any_operation(self):
        eeprom = fake_eeprom(fill=0x00)
        # Simulate stock's own real bytes at page 0.
        stock_bytes = bytes([0x03, 0x01]) + bytes(14)
        eeprom.seek(0)
        eeprom.write(stock_bytes)
        eeprom.flush()

        for i in range(plr.JOURNAL_PAGE_COUNT + 5):
            plr.commit_checkpoint(eeprom, i)
        plr.commit_tombstone(eeprom)
        plr.scan_journal(eeprom)
        plr.read_recovery_state(eeprom)

        eeprom.seek(0)
        self.assertEqual(eeprom.read(plr.PAGE_SIZE), stock_bytes)

    def test_journal_operations_never_address_page_zero(self):
        self.assertEqual(plr.JOURNAL_FIRST_PAGE, 1)
        self.assertNotIn(plr.STOCK_PAGE, range(plr.JOURNAL_FIRST_PAGE,
                                                plr.JOURNAL_LAST_PAGE + 1))


class SidecarParityTests(unittest.TestCase):
    def test_parity_alternates(self):
        self.assertEqual(plr.sidecar_parity(0), 0)
        self.assertEqual(plr.sidecar_parity(1), 1)
        self.assertEqual(plr.sidecar_parity(2), 0)
        self.assertEqual(plr.sidecar_parity(3), 1)

    def test_successive_checkpoints_alternate_sidecar_target(self):
        eeprom = fake_eeprom()
        r1 = plr.commit_checkpoint(eeprom, 1)
        r2 = plr.commit_checkpoint(eeprom, 2)
        r3 = plr.commit_checkpoint(eeprom, 3)
        parities = [plr.sidecar_parity(r.generation) for r in (r1, r2, r3)]
        self.assertEqual(parities, [1, 0, 1])
        self.assertNotEqual(parities[0], parities[1])
        self.assertNotEqual(parities[1], parities[2])


if __name__ == '__main__':
    unittest.main()
