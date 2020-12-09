# Cassie Control - Current Status

## Current Status

There are currently four balancing controllers in this folder. They can be broken into two catagories:

Drake systems based:
* `qp_balance.py` - balancing controller using RPY orientation
* `qp_balance_quat.py` - balancing controller using quaternion orientation
* `qp_balance_lqr.py` - balancing controller using LQR formulation and quaternion orientation

LCM comms based:
* `qp_balance_hw.py` - balancing controller that receives/publishes over LCM and uses RPY orientation

I am working on making the Drake systems based controller work with Lcm Subscriber/Publisher so code doesn't have to be manually mirrored.

## Running
(Note: To visualize any of these you will need to launch a visualizer)

### Lock-step simulation
To run the lock-step simulation run:
```
bazel run //cassie/control:cassie_balancing_sim
```

This will run any of the three Drake systems based controllers with the `version` variable in `cassie_balancing_sim.py` choosing between the them:
* 0 is `qp_balance.py`
* 1 is `qp_balance_quat`
* 2 is `qp_balance_lqr.py`.

### LCM Simulation
To run the test of the controller over Lcm, in one window run:
```
bazel run //cassie/control:qp_balance_hw
```

and in another window run:
```
bazel run //cassie/models:mock_cassie state
```

To adjust the delay, you can uncomment `time.sleep()` and change the delay after each solve before publishing

### Hardware tests
To run a hardware test, you need to run the following, each in their own window:
```
bazel run //cassie/comms:UDPComms
bazel run //cassie/filter:cassie_filter_node
bazel run //cassie/control:qp_balance_hw
```