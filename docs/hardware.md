# Hardware and Vehicle Constraints

## Platform

- Main computer: NVIDIA Jetson Orin Nano
- Flight controller: Pixhawk 2.4.8 with ArduRover
- Vehicle: Ackermann steering RC rover
- Current localization sensor: GPS and compass through MAVROS
- Planned sensors: SLAM sensor and LiDAR; not currently implemented

## Confirmed Vehicle Parameters

| Parameter | Value |
|---|---:|
| Wheelbase | 0.35 m |
| Vehicle width | 0.27 m |
| Vehicle length | 0.50 m |
| Navigation minimum turning radius | 1.7 m |
| Nominal straight speed | 0.60 m/s |
| Curve speed target | about 0.40 m/s |
| ArduRover `WP_SPEED` | 0.60 m/s |
| `SERVO1_MIN` | 1000 PWM |
| `SERVO1_TRIM` | 1500 PWM |
| `SERVO1_MAX` | 2000 PWM |

Pixhawk remains responsible for low-level steering, throttle and actuator output.
Navigation software does not directly command servo PWM.

## Validated Steering Baseline

The current ArduRover steering-rate baseline validated by a fixed LEFT circle
hardware test is:

| Parameter | Value |
|---|---:|
| `ATC_STR_RAT_FF` | 3.0 |
| `ATC_STR_RAT_P` | 1.0 |
| `ATC_STR_RAT_I` | 0.2 |
| `ATC_STR_RAT_D` | 0 |
| `ATC_STR_RAT_IMAX` | 1.0 |
| `ATC_STR_RAT_MAX` | 120 deg/s |
| `ATC_STR_ACC_MAX` | 120 deg/s/s |
| `MOT_SPD_SCA_BASE` | 1.0 m/s |

At a desired yaw rate of 0.235294 rad/s, the final five seconds averaged about
0.2333 rad/s actual yaw rate (about 99% tracking) and a 1.66--1.67 m
speed/yaw-rate radius. The navigation constraint remains 1.7 m. ROS launch
does not write these FCU parameters automatically; verify the FCU values before
hardware operation.
