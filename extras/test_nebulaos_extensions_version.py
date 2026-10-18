# The extensions' own version (G81, 2026-10-18): plain semver, so it can equal
# the repository's vX.Y.Z release tag and the NebulaOS release it ships in.
# nebulaos_compat reports it in klippy.log; nothing may compare it as a
# compatibility gate (that is nebulaos_api_level).
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import json
import os
import re
import unittest

MANIFEST = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                        'nebulaos-extensions.json')


class ExtensionsVersionTest(unittest.TestCase):
    def test_semver(self):
        with open(MANIFEST) as handle:
            version = json.load(handle)['extensions_version']
        self.assertRegex(version, r'^[0-9]+\.[0-9]+\.[0-9]+$')

    def test_listed_in_manifest(self):
        with open(MANIFEST) as handle:
            paths = [m['path'] for m in json.load(handle)['modules']]
        self.assertIn('extras/test_nebulaos_extensions_version.py', paths)


if __name__ == '__main__':
    unittest.main()
