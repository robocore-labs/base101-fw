"""Check real firmware telemetry with the actual Pico SDK formatter on the host.
Usage: python3 tests/calibration_serialization_test.py --pico-sdk /path/to/pico-sdk
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--pico-sdk', default=os.environ.get('PICO_SDK_PATH'))
args = parser.parse_args()
if not args.pico_sdk:
    parser.error('provide --pico-sdk or set PICO_SDK_PATH')
root = Path(__file__).resolve().parents[1]
formatter = Path(args.pico_sdk).resolve() / 'src/rp2_common/pico_printf'
with tempfile.TemporaryDirectory(prefix='base101-formatter-') as directory:
    binary = Path(directory) / 'test'
    sources = ['tests/calibration_drive_test.c', 'calibration_protocol.c', 'calibration_metrics.c', 'glide.c',
               'gyro_yaw.c', 'motion_profile.c', 'odometry.c']
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=undefined',
                    '-Itests/calibration_stubs', '-I.', '-Ilib/pico_serial/include',
                    '-Ilib/pico_ddsm/include', '-Ilib/hardware_link101/include',
                    '-I' + str(formatter / 'include'), *sources, str(formatter / 'printf.c'),
                    '-Wl,--wrap=snprintf', '-Wl,--wrap=vsnprintf', '-lm', '-o', str(binary)],
                   cwd=root, check=True)
    subprocess.run([str(binary)], check=True)
    raw = subprocess.check_output([str(binary), '--json'])
    status = json.loads(raw)
    assert status['protocol'] == 2 and status['brake_ok_mask'] == 15
    assert status['gyro']['calibrated'] and status['ready']
    assert all(wheel['rpm'] == 0 for wheel in status['wheels'])
    assert status['geometry']['separation'] == .2899
    assert len(raw) < 4096
    print(f'Actual Pico formatter telemetry JSON passed ({len(raw)} bytes)')
