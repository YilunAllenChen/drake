import numpy as np
from pydrake.all import (FloatingBaseType, RigidBodyTree, RigidBodyPlant)
from cassie_common import *


def main():
    assert kCassiePositions == 23
    assert kCassieVelocities == 22
    assert kCassieStates == 23+22
    assert kCassieActuators == 10
    assert abs(kCassieFourBarDistance - 0.5012) < 1e-8
    assert p_midfoot.shape == (3,)

    assert CassieURDFType.kStandardCassie == 0
    assert CassieURDFType.kSoftSpringsCassie == 1
    assert CassieURDFType.kActiveSpringsCassie == 2
    assert CassieURDFType.kActiveAnkleCassie == 3
    assert CassieURDFType.kFixedSpringCassie == 4

    assert CassieFixedPointState().shape == (45,)
    assert CassieFixedPointState(FloatingBaseType.kRollPitchYaw).shape == (44,)
    assert CassieFixedPointState(FloatingBaseType.kFixed).shape == (32,)
    assert CassieFixedPointTorque().shape == (10,)
    assert GetFourBarHipMountPoint().shape == (3,)
    assert GetFourBarHeelMountPoint().shape == (3,)

    baseState = BaseStateFromFullState(CassieFixedPointState())
    assert baseState.shape == (13,)

    rtree = getCassieTree()
    assert rtree is not None
    assert getCassieTree(CassieURDFType.kStandardCassie) is not None
    assert getCassieTree(CassieURDFType.kSoftSpringsCassie) is not None
    assert getCassieTree(CassieURDFType.kActiveSpringsCassie) is not None
    assert getCassieTree(CassieURDFType.kActiveAnkleCassie) is not None
    assert getCassieTree(CassieURDFType.kFixedSpringCassie) is not None

    plant = RigidBodyPlant(rtree)
    setDefaultContactParams(plant)


if __name__ == "__main__":
    main()
