import math
from session_1.error_response import simulate_response


def test_zero_i_d_produces_identical_error_traces():
    p,pid=simulate_response((6.5,0.,0.))
    assert p==pid
    assert p[0]==(0.,20.)
    assert p[-1][0]==12.


def test_demo_gains_change_response_without_changing_p_baseline():
    baseline,default=simulate_response((6.5,0.,.8))
    repeated,changed=simulate_response((6.5,.5,2.))
    assert baseline==repeated
    assert default!=changed
    for curve in (baseline,default,changed):
        assert all(math.isfinite(error) for _,error in curve)
    assert abs(default[-1][1])<abs(default[0][1])
