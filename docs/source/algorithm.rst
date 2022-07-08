==========
Algorithm
==========

Tianbotmini is a platform for quick introduction to the theory and knowledge of mobile robots.

Mobile robot kinematics model
==========
Tianbotmini is a two-wheel differential mobile robot, which can be simplified to the motion of a particle and a kinematic model is established.

.. image:: https://github.com/tianbot/abc_swarm/blob/doc/docs/image/001MobileRobotKinematicsModel.png
The kinematic equation of the mobile robot can be expressed as:

.. math::
 \begin{cases}\dot{x}=v\cos(\theta)
 \\\dot{y}=v\sin(\theta)
 \\\dot{\theta}=\omega
 \end{cases}

Multiple mobile robots
==========

Multiple mobile robots exhibit better performance and become a hot field of mobile robot research.

Leader follower formation
----------

The formation of the leader and the follower requires the leader and the follower to maintain a relative positional relationship. We determine the positional relationship by setting relative distance and relative angle. As shown in the image below:

.. image:: https://github.com/tianbot/abc_swarm/blob/doc/docs/image/002LeaderFollowerFormation.png

According to the geometric relationship in the figure, we can get the following equation:



References
==========

.. [1] 
