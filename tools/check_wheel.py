"""Fail closed: this bridge is compiled against the exact installed wheel headers."""
from importlib import metadata
import importlib.util
from pathlib import Path
import runpy
spec=importlib.util.find_spec('nvblox_torch')
if spec is None:raise SystemExit('Install dependencies first: ./install_dependencies.sh')
version=runpy.run_path(str(Path(spec.origin).parent/'_version.py'))
expected='e4a2b00f45ecf989682d72a33a2c29defe39fd2d'
if version.get('__git_sha__')!=expected:
    raise SystemExit('Wheel source commit differs from vendored bridge headers: '+str(version))
import torch
if not torch.cuda.is_available():raise SystemExit('PyTorch CUDA unavailable; verify driver and wheel.')
print('Wheel',metadata.version('nvblox-torch'),'commit',expected,'Torch',torch.__version__,'CUDA',torch.version.cuda)
