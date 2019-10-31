# parameter
- 最終更新日 20191031
## DWA_var.h
|項目                   |変数名         |値     |
|---                    |---            |---    |
|DWA計算のステップ時刻  |DWA.dt         |0.15   |
|軌道計算の刻み         |dt_traj        |0.1    |
|軌道予測時刻           |DWA.PredictTime|3      |
|計算周期               |looprate       |1/dt   |
|                       |k_head         |1      |
|                       |k_vel          |1      |
|                       |thres_vel_time      |2.5  |
|                       |thres_ang_time      |2.5   |
|衝突判定の角度分解能範囲     |POINT_INTEREVAL   |10(points)|
|DWAの分解能の分母            |DWA_RESOLUTION_DIV|5|
|safeがdistの何倍になるか            |LINSAFE_MULTIPLIER|3|
|                                    |ANGSAFE_MULTIPLIER|3|


- thres_vel_time,thres_ang_timeはpredictTimeと同じがよいか?


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
|項目                   |値     |備考|
|---                    |------ |---|
|ロボット質量           |8.0kg  | バッテリ(LRF1.6kg,ロボット5.8kg,車輪0.3kg×2)|
|                       |       ||

車輪のジョイント
```
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <origin rpy="-1.5707 0 0" xyz="0 0.1425 -0.0175"/>
    <axis xyz="0 0 1"/>
    <limit velocity="1.0" effort="2.0"/>	<!--decide mortor power-->
    <dynamics damping="0.2"/>             <!--damping of axes of mortor-->
  </joint>
```

車輪の摩擦特性
```
 <gazebo reference="left_wheel_link">
   <selfCollide>true</selfCollide>
    <mu1 value="9" />
    <mu2 value="9" />
    <kp>500000.0</kp>
    <kd>10.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>0.1</maxVel>
    <fdir1>1 0 0</fdir1>
      <material>Gazebo/Black</material>
  </gazebo>
```

キャスタ特性
```
 <gazebo reference="front_caster">
   <selfCollide>true</selfCollide>
    <mu1 value="0" />
    <mu2 value="0" />
      <material>Gazebo/Green</material>
  </gazebo>
```

laser
```
    <gazebo reference="laser">
      <sensor type="ray" name="${prefix}">
   <!--   <sensor type="gpu_ray" name="laser"> -->
        <pose>0 0 0 0 0 0</pose>
        <visualize>false</visualize>
        <update_rate>10</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>683</samples>--> <!--1024[points]*240[deg]/360[deg]-->
             <resolution>1</resolution>
              <min_angle>${radians(-120)}</min_angle>
              <max_angle>${radians( 120)}</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.06</min>
            <max>4.0</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </ray>
       <plugin name="gazebo_ros_lidar_controller" filename="libgazebo_ros_laser.so">
    <!--  <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_gpu_laser.so"> -->
          <topicName>/scan</topicName>
          <frameName>laser</frameName>
        </plugin>
      </sensor>
```

camera
```
    <gazebo reference="camera">
      <sensor type="depth" name="Kinectv2">
	<always_on>true</always_on>
        <update_rate>30.0</update_rate>

        <camera name="Kinectv2">
          <pose>0 0 0 0 0 0</pose>
          <horizontal_fov>1.221730</horizontal_fov> <!--viewing angle 70degree-->
         <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.01</near>
            <far>8.0</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.004</stddev> 
          </noise>
        </camera>
```

## my_robo_spec.h
|項目                   |変数名         |値     |
|---                    |---            |---    |
|ロボットの横幅         |ROBOT_WIDTH    |0.4  |
|縦幅                   |ROBOT_LENGTH   |0.3    |
|衝突半径 |ROBOT_RAD|sqrt((ROBOT_WIDTH * ROBOT_WIDTH)/4 + (ROBOT_LENGTH* ROBOT_LENGTH)/4)    |
