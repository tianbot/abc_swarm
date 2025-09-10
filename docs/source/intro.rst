Introduction
============

ABC Swarm is an open source project short for Ant Bee Cooperative Swarm. This project aims to use low-cost  robots and drones to achieve multi-robot experiment sulotion without external positioning system. The overall objective of this framework is to support multi-robot coordination, air-ground cooperative swarm, formation control, ant colony optimization, wolf pack algorithm, distributed optimization, reinforcement deep learning, etc.

1. formation_publisher
----------------------

- publish `long snake` formation
- publish `circle` formation
- publish `triangle` formation
- publish `herringbone` formation

1. path generator
------------------

- generate path with `draw` in browser
- generate path with `cubic_spline` algorithm with `waypoint` input

1. target_tracker
-----------------

-  track target with `PID` controller
-  track path with `Pure Pursuit` controller

4. vrpn_alive_monitor
----------------------

- monitor `MotionCapture` vrpn stream with `ping` and `topic echo`

5. vel_utils
-------------

- convert `/cmd_vel_global` to `/cmd_vel_body`