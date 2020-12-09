import numpy as np
# from pydrake.all import (RigidBodyTree, RigidBody)
from cassie_lcm_parsing import *


def main():
    assert len(kCassieMotorToState) == 10
    assert len(kCassieJointToState) == 4
    assert len(kCassieSpringToState) == 4
    assert len(kCassieMotorUrdfToSimulink) == 10


if __name__ == "__main__":
    main()
