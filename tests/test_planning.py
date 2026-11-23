import numpy as np
from urc_core.planning import Grid, lawnmower, select_bin


def test_astar_routes_around_blocked_cells():
    grid=Grid([0,5,0,5],1.)
    grid.blocked={(2,j) for j in range(4)}
    points=grid.path([.5,.5],[4.5,.5]); cells=[grid.cell(p) for p in points]
    assert not grid.blocked.intersection(cells)
    a=np.array([.5,.5])
    for b in points:
        assert not any(grid.cell(a+(b-a)*f) in grid.blocked for f in np.linspace(0,1,120))
        a=b
    assert cells[-1]==(4,0)
    grid.blocked.add((2,4))
    assert grid.path([.5,.5],[4.5,.5])==[]


def test_bin_policies_and_capacity():
    bins={'bin_a':np.array([-2,2,0]),'bin_b':np.array([-2,-2,0])}
    rule={'bottle':'bin_a','hammer':'bin_b'}
    at=np.array([-2,-1.,0])
    assert select_bin('by_class','bottle',at,bins,{},6,rule,'bin_b')=='bin_a'
    assert select_bin('nearest','bottle',at,bins,{},6,rule,'bin_a')=='bin_b'
    assert select_bin('nearest','bottle',at,bins,{'bin_b':6},6,rule,'bin_b')=='bin_a'
    assert select_bin('ui_select','bottle',at,bins,{},6,rule,'bin_b')=='bin_b'
    assert select_bin('by_class','bottle',at,bins,{'bin_a':6},6,rule,'bin_b') is None
    assert lawnmower([4.,9.,-3.,3.],2.)[:4]==[(4.3,-2.),(8.7,-2.),(8.7,0.),(4.3,0.)]
