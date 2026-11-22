# EKF

at a high level here is roughly how EKF works:

Let's say that you have several different lower quality IMUs and other kinds of sensors that can be used to roughly estimate your
position/velocity/acceleration.
However, those different kinds of sensors & IMUs all have different forms of errors (for example, one might be really good at determining velocity while moving,
but is inaccurate while stationary).
EKF allows you to take all of those lower quality sensors and merge them together into a single higher quality sensor.

## Local vs Global

The robot runs 2 EKF nodes.

The first EKF node is for a local EKF, and the second one is for a global EKF.
