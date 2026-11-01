# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Compile and execute the real callback/write methods without a ROS runtime."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class CvTargetBoundaryTest(unittest.TestCase):
    def test_production_methods(self):
        source = (ROOT / 'src/dji_serial_bridge_node.cpp').read_text()
        methods = []
        for signature in ['  bool send_frame(', '  void cv_target_callback(']:
            start = source.index(signature)
            end = source.index('\n  }', start) + len('\n  }')
            methods.append(source[start:end])
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            (directory / 'bridge_write_methods.inc').write_text('\n'.join(methods))
            binary = directory / 'boundary-test'
            subprocess.run([
                os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra',
                '-Werror', '-pthread', '-I' + str(ROOT / 'include'), '-I' + tmp,
                str(ROOT / 'test/cv_target_boundary.cpp'), str(ROOT / 'src/crc_dji.cpp'),
                '-o', str(binary),
            ], check=True, timeout=60)
            subprocess.run([str(binary)], check=True, timeout=10)


if __name__ == '__main__':
    unittest.main()
