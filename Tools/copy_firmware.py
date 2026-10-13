"""PlatformIO post-build script copying the compiled firmware to Build/.

Files are named as in the Sanworks Firmware Loader: <name>_v<version>.<extension>,
e.g. StateMachine_Bpod2Plus_v023.hex. The name is set per environment with the
`custom_firmware_name` option in platformio.ini (default: the environment name);
the version is the major firmware version of the sketch.
"""

import re
import shutil
from pathlib import Path

Import('env')  # noqa: F821  # SCons global provided by PlatformIO


def firmware_version() -> int:
    """Return FIRMWARE_VERSION_MAJOR as defined in the sketch."""
    sketch = next(Path(env.subst('$PROJECT_SRC_DIR')).glob('*.ino'))  # noqa: F821
    # only the version line matters, so tolerate bytes that are not valid UTF-8
    source = sketch.read_text(encoding='utf-8', errors='replace')
    match = re.search(r'^#define\s+FIRMWARE_VERSION_MAJOR\s+(\d+)', source, re.M)
    if match is None:
        raise ValueError(f'FIRMWARE_VERSION_MAJOR not found in {sketch}')
    return int(match.group(1))


def copy_firmware(source, target, env) -> None:  # noqa: ARG001  # target is unused
    """Copy the firmware image to Build/<name>_v<version>.<extension>."""
    if env.GetBuildType() != 'release':
        print(f'Not copying {env.GetBuildType()} build to Build/')
        return
    # the source is the image to upload: .hex on Teensy, .bin on Arduino Due
    firmware = Path(source[0].get_abspath())
    name = env.GetProjectOption('custom_firmware_name', env['PIOENV'])
    build_dir = Path(env.subst('$PROJECT_DIR')) / 'Build'
    build_dir.mkdir(exist_ok=True)
    destination = build_dir / f'{name}_v{firmware_version():03d}{firmware.suffix}'
    shutil.copy2(firmware, destination)
    print(f'Copied firmware to {destination}')


# The platform's 'buildprog' target has the firmware image as its source. Copy it
# on every build and before every upload, also when the image is already up to date.
image = env.Alias('buildprog')[0].sources  # noqa: F821
copy = env.AlwaysBuild(env.Alias('copyfirmware', image, copy_firmware))  # noqa: F821
env.Default(copy)  # noqa: F821
env.Depends('upload', copy)  # noqa: F821
