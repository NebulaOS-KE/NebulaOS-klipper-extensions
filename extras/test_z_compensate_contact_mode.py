# G99 (Tim 2026-10-22): the nozzle clean follows the user's load cell mode
# like every other nozzle touch - automatic (raw counts, ke-stock-v1) by
# default, "also automatically, untested". _NEBULAOS_NOZZLE_CLEAN refuses
# BEFORE heating or moving when the touch would be refused anyway (manual
# without the bottle calibration, automatic without a passed check); the
# touches themselves get no plan from nozzle_clear and are resolved by the
# probe from the mode (test_nebulaos_raw_contact.RawArmingTest
# test_touch_without_a_plan_follows_auto_mode).
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import unittest
from unittest import mock

from . import prtouch_test_support as fake
from . import nebulaos_raw_contact as rc
from . import z_compensate


class ModeProbe(fake.FakeZOffsetProbe):
    def __init__(self, mode, calibrated=False, reference=None, failed=False,
                 refresh_ok=True):
        super().__init__()
        self.mode, self.calibrated = mode, calibrated
        self.reference, self.failed = reference, failed
        self.plans = []
        self.refresh_ok = refresh_ok
        self.refreshes = 0

    def refresh_auto_reference(self):
        self.refreshes += 1
        if self.mode != 'auto':
            return True, ''
        if self.refresh_ok:
            return True, ''
        self.failed = True
        return False, 'the sensor reading does not change at all'

    def plan_arming(self, caller, profile=None):
        self.plans.append(caller)
        return rc.resolve_arming(self.mode, caller, self.calibrated,
                                 rc.DEFAULT_PROFILE_ID, self.reference,
                                 auto_check_failed=self.failed)


def _build(probe):
    printer, mcu, _pins, _values = fake.build_environment()
    printer.objects['nebulaos_z_offset_probe'] = probe
    zc_config = fake.make_z_compensate_config(
        printer, dict(fake.REAL_Z_COMPENSATE_CONFIG))
    zc = z_compensate.ZCompensate(zc_config)
    fake.connect(printer, mcu)
    return printer, zc


class NozzleCleanFollowsTheModeTest(unittest.TestCase):
    def _run(self, probe):
        printer, zc = _build(probe)
        with mock.patch.object(z_compensate.nozzle_clear,
                               'clear_nozzle') as clear:
            zc.cmd_nozzle_clear(fake.FakeGCmd({}))
        return clear

    def _refused(self, probe):
        printer, zc = _build(probe)
        with mock.patch.object(z_compensate.nozzle_clear,
                               'clear_nozzle') as clear:
            with self.assertRaises(fake.CommandError) as ctx:
                zc.cmd_nozzle_clear(fake.FakeGCmd({}))
        clear.assert_not_called()           # nothing heated, nothing moved
        return str(ctx.exception)

    def test_auto_mode_with_a_passed_check_cleans(self):
        probe = ModeProbe('auto', reference=-240426)
        clear = self._run(probe)
        clear.assert_called_once()
        self.assertEqual(probe.plans, [rc.CALLER_NOZZLE_CLEAN])
        self.assertEqual(probe.plan_arming(rc.CALLER_TOUCH)[0], rc.ARM_RAW)

    def test_auto_mode_needs_no_bottle_calibration(self):
        clear = self._run(ModeProbe('auto', calibrated=False,
                                    reference=-240426))
        clear.assert_called_once()

    def test_auto_mode_without_a_check_refuses_clearly(self):
        msg = self._refused(ModeProbe('auto', calibrated=True))
        self.assertIn('NEBULAOS_LOAD_CELL_AUTO_CHECK', msg)
        self.assertIn('Manual', msg)

    def test_auto_mode_after_a_failed_check_refuses(self):
        msg = self._refused(ModeProbe('auto', failed=True,
                                      reference=-240426))
        self.assertIn('failed', msg)

    def test_manual_mode_uses_the_calibration(self):
        probe = ModeProbe('manual', calibrated=True)
        self._run(probe).assert_called_once()
        self.assertEqual(probe.plan_arming(rc.CALLER_TOUCH),
                         (rc.ARM_CALIBRATED, None))

    def test_manual_mode_without_the_bottle_calibration_refuses(self):
        msg = self._refused(ModeProbe('manual', calibrated=False,
                                      reference=-240426))
        self.assertIn('NEBULAOS_LOAD_CELL_CALIBRATE', msg)

    def test_the_reference_is_refreshed_hot_before_the_first_touch(self):
        probe = ModeProbe('auto', reference=-240426)
        clear = self._run(probe)
        refresh = clear.call_args.kwargs['before_first_touch']
        self.assertEqual(probe.refreshes, 0)   # not before the heating
        refresh()
        self.assertEqual(probe.refreshes, 1)

    def test_a_failed_hot_refresh_stops_with_the_manual_offer(self):
        probe = ModeProbe('auto', reference=-240426, refresh_ok=False)
        refresh = self._run(probe).call_args.kwargs['before_first_touch']
        with self.assertRaises(fake.CommandError) as ctx:
            refresh()
        self.assertIn('does not change', str(ctx.exception))
        self.assertIn('Manual', str(ctx.exception))

    def test_clear_nozzle_refreshes_after_heating_before_touching(self):
        import inspect
        from . import nozzle_clear
        src = inspect.getsource(nozzle_clear.clear_nozzle)
        call = src.index('before_first_touch()')
        self.assertLess(src.index('wait=True'), call)          # heated
        self.assertLess(call, src.index('src_pos[2] = _touch_probe_no_mesh'))

    def test_a_probe_without_modes_is_unchanged(self):
        printer, zc = _build(fake.FakeZOffsetProbe())
        with mock.patch.object(z_compensate.nozzle_clear,
                               'clear_nozzle') as clear:
            zc.cmd_nozzle_clear(fake.FakeGCmd({}))
        clear.assert_called_once()


if __name__ == '__main__':
    unittest.main()
