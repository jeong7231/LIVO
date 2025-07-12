# 1. SLAM

## 1. 실행 명령어

```bash
# 1. 센서 및 기본 오도메트리 실행
ros2 launch livo_bringup livo_odom.launch.py

# 2. 모터 드라이버 및 조이스틱 제어 노드 실행
ros2 launch odrive_ep_pkg odrive_ep.launch.py
ros2 run joy joy_node

# 3. SLAM 알고리즘 실행
ros2 launch livo_navigation slam.launch.py rviz:=true sim:=false
```


## 1.2 단계별 설명

1.  **센서/구동계 활성화**: `livo_odom.launch.py`가 실행되어 Lidar, Realsense 카메라, IMU 센서 및 모터 엔코더를 이용한 기본 오도메트리 노드(`odom_node`)가 활성화된다.
2.  **수동 조작 준비**: `joy_node`는 조이스틱 입력을 `/joy` 토픽으로 발행하고, `odrive_ep_pkg`의 `odrive_ep_joy` 노드는 이를 구독하여 모터 제어 명령인 `/goal_posvel_double`로 변환한다. `odrive_ep_motor_double` 노드가 이 명령을 받아 실제 로봇을 움직인다.
3.  **SLAM 실행**: `slam.launch.py`가 SLAM 알고리즘 노드를 실행합한다. 이 노드는 `/scan`, `/imu/data`, `/odom` 등 모든 센서 데이터를 구독하여 지도를 생성(`map` 토픽)하고, `map` 프레임과 `odom` 프레임 간의 관계(`tf`)를 지속적으로 계산하여 발행한다.
4.  **지도 작성**: 사용자는 조이스틱으로 로봇을 움직여 환경을 탐색하고, SLAM 노드는 로봇의 이동 경로와 센서 데이터를 바탕으로 지도를 완성해나갑니다. RViz를 통해 이 과정이 시각화된다.

---

# 2. Navigation

## 2.1 실행 명령어

```bash
# 1. 센서 및 기본 오도메트리 실행
ros2 launch livo_bringup livo_odom.launch.py

# 2. 모터 드라이버 실행
ros2 launch odrive_ep_pkg odrive_ep.launch.py

# 3. 위치 추정 강화 (Visual Odometry)
ros2 launch livo_navigation rtab_color_livo.launch.py

# 4. Navigation 스택 실행 (Nav2)
ros2 launch livo_navigation navigation.launch.py rviz:=true sim:=false
```

## 2.2 단계별 설명

1.  **기반 시스템 활성화**: `livo_bringup`과 `odrive_ep_pkg`가 실행되어 센서와 모터가 준비된다.
2.  **위치 추정 강화**: `rtab_color_livo.launch.py`가 실행되어 Realsense 카메라 기반의 Visual Odometry(VO)가 활성화된다. 이는 Lidar나 IMU만 사용하는 것보다 더 정확한 위치 추정을 돕는다.
3.  **Navigation 스택 실행**: `navigation.launch.py`가 Nav2 스택을 실행한다.
    *   **Map Server**: 미리 생성된 지도를 불러와 `/map` 토픽으로 발행한다.
    *   **Localization (AMCL)**: Lidar(`scan`), IMU, 오도메트리, VO 등 모든 위치 관련 정보를 종합하여, 불러온 지도 위에서 로봇의 현재 위치를 매우 정확하게 추정한다.
    *   **BT Navigator, Planner, Controller**: 사용자가 RViz에서 목표 지점을 설정하면(`goal_pose`), BT Navigator가 경로 계획(Planner)과 경로 추종(Controller)을 총괄하여 로봇의 이동을 제어한다.
4.  **자율 주행**: Controller는 계산된 경로를 따라가기 위해 속도 명령(`cmd_vel`)을 발행한다. `livo_bringup`의 `livo_driver` 노드가 이 `cmd_vel`을 ODrive가 이해할 수 있는 `/goal_posvel_double` 메시지로 변환하여 로봇이 목표 지점까지 자율적으로 이동한다.

---
