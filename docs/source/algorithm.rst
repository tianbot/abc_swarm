==========
Algorithm
==========

Tianbotmini is a platform for quick introduction to the theory and knowledge of mobile robots.

Mobile Robot Kinematics Model
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
Leader follower formation
----------
The formation control algorithms of multi-mobile robots mainly include pilot following method, artificial potential field method, behavior method and virtual rigid body method.
The form of the pilot-following formation control algorithm is simple, and the mobile robot is easy to form a formation. The pilot-following formation control algorithm can be divided into the method based on distance and angle and the method based on distance and distance.


References
==========

.. [1] 
