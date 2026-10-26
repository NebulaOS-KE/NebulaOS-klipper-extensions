# Tests for nebulaos_shaper_graph.py (G109, Run 20): reads upstream's
# SHAPER_CALIBRATE CSV and draws the screen + Mainsail PNGs.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest

from . import nebulaos_shaper_graph as graph

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      'nebulaos_shaper_graph.py')


def write_csv(path, shapers=("zv(59.0)", "mzv(52.4)")):
    with open(path, "w") as f:
        f.write("freq,psd_x,psd_y,psd_z,psd_xyz")
        if shapers:
            f.write(",shapers:," + ",".join(shapers))
        f.write("\n")
        fr = 5.0
        while fr < 133.0:
            p = 1e3 / (1 + ((fr - 48.0) / 4.0) ** 2) + 30.0
            f.write("%.1f,%.3e,%.3e,%.3e,%.3e" % (fr, p * .8, p * .1, p * .1, p))
            if shapers:
                f.write(",," + ",".join("%.3f" % (1.0 / (1 + (fr / 50.0) ** 2),)
                                        for _ in shapers))
            f.write("\n")
            fr += 0.2


def png_size(path):
    with open(path, 'rb') as f:
        head = f.read(24)
    assert head[:8] == b'\x89PNG\r\n\x1a\n'
    return struct.unpack('>II', head[16:24])


def have_matplotlib():
    try:
        import matplotlib  # noqa: F401
        return True
    except ImportError:
        return False


class ReadCsvTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp, True)

    def test_reads_psd_and_every_shaper(self):
        p = os.path.join(self.tmp, 'c.csv')
        write_csv(p)
        freqs, psd, shapers = graph.read_csv(p)
        self.assertAlmostEqual(freqs[0], 5.0)
        self.assertEqual(len(freqs), len(psd))
        self.assertEqual(sorted(shapers), ['mzv', 'zv'])
        self.assertEqual(shapers['mzv'][0], 52.4)
        self.assertEqual(len(shapers['mzv'][1]), len(freqs))

    def test_without_shapers(self):
        p = os.path.join(self.tmp, 'c.csv')
        write_csv(p, shapers=())
        self.assertEqual(graph.read_csv(p)[2], {})

    def test_refuses_another_file(self):
        p = os.path.join(self.tmp, 'c.csv')
        with open(p, 'w') as f:
            f.write("time,accel_x,accel_y,accel_z\n0,1,2,3\n")
        with self.assertRaises(ValueError):
            graph.read_csv(p)


class MainTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp, True)

    def run_script(self, *args):
        env = dict(os.environ, MPLCONFIGDIR=os.path.join(self.tmp, 'mpl'))
        return subprocess.run([sys.executable, SCRIPT] + list(args), env=env,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              timeout=120)

    def test_usage(self):
        r = self.run_script('only-one-arg')
        self.assertEqual(r.returncode, 2)

    def test_missing_csv_is_one_line_on_stderr(self):
        r = self.run_script(os.path.join(self.tmp, 'nope.csv'), 'x', 'mzv', '52.4',
                            os.path.join(self.tmp, 's.png'))
        self.assertEqual(r.returncode, 1)
        self.assertEqual(len(r.stderr.decode().strip().splitlines()), 1)
        self.assertFalse(os.path.exists(os.path.join(self.tmp, 's.png')))

    @unittest.skipUnless(have_matplotlib(), "matplotlib not installed")
    def test_draws_both_pngs_atomically(self):
        csv = os.path.join(self.tmp, 'c.csv')
        write_csv(csv)
        screen = os.path.join(self.tmp, 'cal', 'input_shaper_x.png')
        big = os.path.join(self.tmp, 'config', 'input_shaper', 'shaper_x.png')
        r = self.run_script(csv, 'x', 'mzv', '52.4', screen, big)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(png_size(screen), (480, 272))
        self.assertEqual(png_size(big), (1200, 700))
        leftovers = [f for d in (os.path.dirname(screen), os.path.dirname(big))
                     for f in os.listdir(d) if f.endswith('.tmp')]
        self.assertEqual(leftovers, [])


if __name__ == '__main__':
    unittest.main()
