# CrazySLAM

**CrazySLAM** implements a SLAM algorithm using ultrasound range inputs to
localize the Crazyflie drone.
The project was conducted as part of my engineering degree at INSA Rouen
Normandie.

## Some background
The localization stack is the most important module in autonomous, mobile
systems. Where in some cases the solution to this problem is pretty
straightforward (applications with a high inaccuracy tolerance and high prior
knowledge of an outdoor environment), it can get really difficult for some kind
of devices.

One of the biggest challenges in localization is when we operate in an indoor
environment. Because we can't use GPS sensors, some of the remaining options
are :
  1. Using cheap/inaccurate sensors (mainly IMUs) but they tend to drift, which
  could lead to huge errors.
  2. Using high-end sensors (stereo cameras, LIDAR, depth cameras, etc.) which
  are accurate but needs significantly more processing power and can't be
  implemented on a small-sized vehicle.
  3. Using prior knowledge of the environment (map, set points), which is a good
  idea only for very specific applications (robot in a warehouse).

None of these options were good enough for the Crazyflie:
  1. The Crazyflie framework uses IMUs with a Kalman Filter but the position
  is inaccurate and tends to drift even more.
  2. High end sensors can't be implemented on such a small scale drone: Not
  powerful enough to lift a heavy sensor. Batteries not powerful enough to
  sustain a heavy load. Not enough processing power onboard. Radio bandwidth
  not big enough for high frequency data transmission to a ground station.
  3. Could use the [Loco Positioning system](bitcraze.io/loco-pos-system)
  but doesn't scale as you need to install it in the environment where the drone
  will operate.

## SLAM algorithms

## Implementation

### Mapping

### Localization

## Install

## Contributing guidelines
