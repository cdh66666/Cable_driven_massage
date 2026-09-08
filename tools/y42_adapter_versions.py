"""Load a fresh, immutable diagnostic adapter at a new worker boundary.

Never reload the web process or its state, serial lock, or existing workers.
Only this allowlisted adapter is versioned; dependencies need normal deployment.
"""
import hashlib
import sys
import types
from pathlib import Path


def load_microprobe_adapter():
    path = Path(__file__).with_name('y42_position_microprobe_io.py')
    source = path.read_bytes()
    digest = hashlib.sha256(source).hexdigest()
    name = '_y42_microprobe_' + digest
    if name in sys.modules:
        return sys.modules[name], digest
    code = compile(source, str(path), 'exec')
    module = types.ModuleType(name)
    module.__file__ = str(path)
    sys.modules[name] = module
    try:
        exec(code, module.__dict__)
        if not callable(getattr(module, 'run_hardware_position_microprobe', None)):
            raise TypeError('Microprobe adapter entry point missing')
    except BaseException:
        sys.modules.pop(name, None)
        raise
    return module, digest


def load_corner_probe_adapter():
    """Load the explicitly allowlisted bounded corner diagnostic at idle start."""
    path = Path(__file__).with_name('y42_corner_stop_probe.py')
    source = path.read_bytes()
    digest = hashlib.sha256(source).hexdigest()
    name = '_y42_corner_probe_' + digest
    if name in sys.modules:
        return sys.modules[name], digest
    module = types.ModuleType(name)
    module.__file__ = str(path)
    sys.modules[name] = module
    try:
        exec(compile(source, str(path), 'exec'), module.__dict__)
        if not callable(getattr(module, 'run_hardware_corner_stop_probe', None)):
            raise TypeError('Corner probe adapter entry point missing')
    except BaseException:
        sys.modules.pop(name, None)
        raise
    return module, digest
