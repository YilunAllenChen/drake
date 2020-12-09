import numpy as np
from pydrake.all import (RigidBodyTree, RigidBody)
from cassie_common import *
from control_util import *


def main():
    nc = 4
    assert mu == 1.0
    assert m_surface_tangents == 4

    rtree = getCassieTree()
    q = rtree.getZeroConfiguration()
    v = np.zeros(kCassieVelocities)
    cache = rtree.doKinematics(q, v)
    lfoot = rtree.FindBody("toe_left")
    rfoot = rtree.FindBody("toe_right")

    pts = forwardKinTerrainPoints(rtree, cache, lfoot, rfoot)
    Jp_qd = contactJacobian(rtree, cache, lfoot, rfoot, True)
    Jp_v = contactJacobian(rtree, cache, lfoot, rfoot, False)
    Jpdot_times_v = contactJacobianDotTimesV(rtree, cache, lfoot, rfoot)
    assert pts.shape == (3, nc)
    assert Jp_qd.shape == (3*nc, kCassiePositions)
    assert Jp_v.shape == (3*nc, kCassieVelocities)
    assert Jpdot_times_v.shape == (3*nc,)

    normal = np.array([0, 0, 1])
    tangets = surfaceTangents(normal)
    assert tangets.shape == (3, 4)

    phi, normals = contactDistancesAndNormals(rtree, cache, lfoot, rfoot)
    B_q, JB_q = contactJacobianBV(rtree, cache, lfoot, rfoot, True)
    B_v, JB_v = contactJacobianBV(rtree, cache, lfoot, rfoot, False)
    assert phi.shape == (nc,)
    assert normals.shape == (3, nc)
    assert B_q.shape == (3, 4*nc)
    assert JB_q.shape == (kCassiePositions, 4*nc)
    assert B_v.shape == (3, 4*nc)
    assert JB_v.shape == (kCassieVelocities, 4*nc)


if __name__ == "__main__":
    main()
