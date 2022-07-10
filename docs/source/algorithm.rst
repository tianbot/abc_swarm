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

Paths and trajectories
==========
The path of the mobile robot is only related to the position, and the trajectory is not only related to the position, but also constrained by time.

Multiple mobile robots
==========

Multiple mobile robots exhibit better performance and become a hot field of mobile robot research.

Leader follower formation
----------

The formation of the leader and the follower requires the leader and the follower to maintain a relative positional relationship. We determine the positional relationship by setting relative distance and relative angle. As shown in the image below:

.. image:: https://github.com/tianbot/abc_swarm/blob/doc/docs/image/002LeaderFollowerFormation.png

According to the geometric relationship in the figure, we can get the following equation:

.. math::
 \begin{cases}l_{x}=\left(x_{L}-x_{F}-d\cos\left(\theta_{F}\right)\right)\cos\left(\theta_{L}\right)+\left(y_{L}-y_{F}-d\sin\left(\theta_{F}\right)\right)\sin\left(\theta_{L}\right)
 \\l_{y}=-\left(x_{L}-x_{F}-d\cos\left(\theta_{F}\right)\right)\sin\left(\theta_{L}\right)+\left(y_{L}-y_{F}-d\sin\left(\theta_{F}\right)\right)\cos\left(\theta_{L}\right)
 \end{cases}

We need to know how the error changes, after derivation of the above formula, we can get the following equation:
Here, we use a control law that linearizes the input and output.

Formation change
----------
When multiple mobile robots move in a certain formation, they will switch formations according to the needs of the task. The following are common formations.

References
==========

.. [1] 
