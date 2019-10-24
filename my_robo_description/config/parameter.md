# parameter
- 最終更新日 20191023
## DWA_var.h
|項目                   |変数名         |値     |
|---                    |---            |---    |
|DWA計算のステップ時刻  |DWA.dt         |0.15   |
|軌道計算の刻み         |dt_traj        |0.1    |
|軌道予測時刻           |DWA.PredictTime|3      |
|計算周期               |looprate       |6.6667      |
|                       |k_head         |1      |
|                       |k_vel          |1      |
|                       |thres_vel_time      |3      |
|                       |thres_ang_time      |3      |
- thres_vel_time,thres_ang_timeはpredictTimeと同じがよいか?


## my_robo_drive.cpp
|項目                   |変数名         |値     |
|---                    |---            |---    |
|衝突判定の角度         |sensor::cal_obsの引数 |4度ずつ80度|
|DWAの候補速度の刻み    |set_resolutionの引数|acc*dt/5|

## Shared_DWA.cpp
|項目                   |変数名         |値     |
|---                    |---            |---    |
|衝突判定のしきい値     |cal_Dist2()中の変数threshold |0.3|
|    |set_resolutionの引数|acc*dt/4|

## controller.yaml
```
my_robo:
  diff_drive_controller:
    type        : "diff_drive_controller/DiffDriveController"
    left_wheel  : 'left_wheel_joint'
    right_wheel : 'right_wheel_joint'
    publish_rate: 10.0               # default: 50
    wheel_separation : 0.285
    wheel_radius : 0.075
    pose_covariance_diagonal : [0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1000.0]
    twist_covariance_diagonal: [0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1000.0]

    # Wheel separation and diameter. These are both optional.
    # diff_drive_controller will attempt to read either one or both from the
    # URDF if not specified as a parameter

    # Wheel separation and radius multipliers
    # wheel_separation_multiplier: 1.0 # default: 1.0
    # wheel_radius_multiplier    : 1.0 # default: 1.0

    # Velocity commands timeout [s], default 0.5
    cmd_vel_timeout: 10.0

    # Base frame_id
    base_frame_id: base_footprint
    # default: base_link
    odom_frame_id: odom

    # Velocity and acceleration limits
    # Whenever a min_* is unspecified, default to -max_*
    linear:
      x:
        has_velocity_limits    : true
        max_velocity           : 0.8  # m/s
        min_velocity           : -0.8 # m/s
        has_acceleration_limits: true
        max_acceleration       : 1.0  # m/s^2
        min_acceleration       : -1.0 # m/s^2
    angular:
      z:
        has_velocity_limits    : true
        max_velocity           : 1.5  # rad/s
        min_velocity           : -1.5
        has_acceleration_limits: true
        max_acceleration       : 1.5  # rad/s^2
        min_acceleration       : -1.5
```

## my_robo.xacro
|項目                   |変数名         |値     |
|---                    |---            |---    |
|                       |               |       |
|                       |               |       |

## my_robo_spec.h
|項目                   |変数名         |値     |
|---                    |---            |---    |
|ロボットの横幅         |robot_width    |0.4  |
|縦幅                   |robot_length   |0.3    |

## MyDWA.h
|項目                   |変数名         |値     |
|---                    |---            |---    |
|d_scaling         |d_scaling     |1  |
|th_scaling                   |th_scaling     |1    |
|
- スケーリングは何がいいのかわからない