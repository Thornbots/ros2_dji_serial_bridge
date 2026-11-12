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
"""Exercise the production parser and opaque handler with a minimal ROS shim."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class DebugParserTest(unittest.TestCase):
    def test_production_parser(self):
        source = (ROOT / 'src/dji_serial_bridge_node.cpp').read_text()
        methods = []
        for signature in ['  void process_rx_buffer(', '  rclcpp::Time frame_stamp(',
                          '  void dispatch_incoming(', '  void handle_debug(']:
            start = source.index(signature)
            end = source.index('\n  }', start) + len('\n  }')
            methods.append(source[start:end])
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            (directory / 'bridge_debug_methods.inc').write_text('\n'.join(methods))
            binary = directory / 'debug-test'
            subprocess.run([
                os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra',
                '-Werror', '-I' + str(ROOT / 'include'), '-I' + tmp,
                str(ROOT / 'test/debug_parser.cpp'), str(ROOT / 'src/crc_dji.cpp'),
                '-o', str(binary),
            ], check=True, timeout=60)
            subprocess.run([str(binary)], check=True, timeout=10)


if __name__ == '__main__':
    unittest.main()
