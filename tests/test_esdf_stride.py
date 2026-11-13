"""Stride is a display/export choice, not a change to integration resolution."""
import numpy as np
import pytest
from ur7e_nvblox_in_hand.node import DEFAULTS, validate
from ur7e_nvblox_in_hand.backend import Backend

@pytest.mark.parametrize('stride', [0, -1, True, 1.5])
def test_invalid_stride_rejected(stride):
    cfg = dict(DEFAULTS, esdf_view_stride=stride)
    with pytest.raises(ValueError, match='stride'):
        validate(cfg)

def test_backend_keeps_dense_export_and_dispatches_strided_export():
    calls = []
    class Tensor:
        def numpy(self): return np.ones((1, 7), np.float32)
    class Ops:
        def export_esdf(self, *args): calls.append(('dense', args)); return Tensor()
        def export_esdf_strided(self, *args): calls.append(('strided', args)); return Tensor()
    class Mapper:
        def get_c_mapper(self): return 'mapper'
    backend = Backend.__new__(Backend)
    backend.ops, backend.mapper = Ops(), Mapper()
    backend.esdf('observed', .24)
    backend.esdf('observed', .24, 2)
    assert calls == [('dense', ('mapper', 0, .24)), ('strided', ('mapper', 0, .24, 2))]
