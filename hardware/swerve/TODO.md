# TODO

## General

- [ ] Unit tests

## Control

- [ ] Implement command velocity timeout
- [ ] Per-wheel velocity limits
    - When limiting the velocity of one wheel, all the other wheels must be limited by the same factor.
    - Doing this might be sub-optimal, so maybe just limits on input velocity is sufficient?
- [ ] Stop wheels on controller deactivation
- [ ] Reset buffers on controller deactivation

## Odometry

- [ ] Open-loop odometry
- [ ] Publish odometry message
- [ ] Publish odometry to `/tf`
- [ ] Closed-loop odometry
- [ ] Analyze integration methods
    - The differential-drive controller uses:
        - linear approximation for small changes
        - RK2 for large changes
- [ ] Get wheel offsets from tf2 instead of from a config file

## Nav2

See: https://arxiv.org/pdf/2510.19054

- [ ] Swerve-constraint critic
