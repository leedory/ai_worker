# SG2 1050 inference bringup

이 문서는 `ffw-SNPR48A1050`에서 `ai_worker`가 담당하는 실제 로봇
bringup과 Cyclo Intelligence 사이의 인터페이스만 설명한다. 모델, checkpoint,
22D state/action 조립과 policy 후처리는 `cyclo_intelligence`의 책임이다.

## 실행 구조

```text
SG2 sensors/controllers
  -> ai_worker (ROS 2 publishers and controllers)
  -> ROS 2 topics through the host-network Zenoh runtime
  -> cyclo_intelligence (observation, inference, command publisher)
  -> ai_worker command topics
  -> SG2 controllers
```

2026-09-11 읽기 전용 확인 기준으로 1050에는 다음 컨테이너가 함께 실행된다.

- `ai_worker`: `robotis/ai-worker:2.2.1`
- `cyclo_intelligence`: `robotis/cyclo-intelligence:1.2.1`
- `zenoh_router`: loopback `tcp/127.0.0.1:7447`, shared memory enabled

`ai_worker`는 host network를 사용한다. 호스트 저장소
`/home/robotis/ai_worker`가 컨테이너의
`/root/ros2_ws/src/ai_worker`에 bind mount되므로, image tag만 보고 실제 source
revision을 판단하면 안 된다.

5090 workstation은 이 브랜치를 개발·검증하고 Git/SSH로 배포를 준비하는
장소다. 현재 확인된 1050 구성에서는 추론도 1050의
`cyclo_intelligence` 컨테이너에서 실행되며, 5090이 정상 운용 중 영상이나
명령을 중계하지 않는다. offboard inference를 추가한다면 연결·재시도·보안
설정은 `cyclo_intelligence` 문서에서 별도의 runtime topology로 정의해야 한다.

## ai_worker가 제공하는 계약

1050에서 검증한 카메라 입력은 다음과 같다.

| 역할 | 장치 | ROS image topic | 획득 크기 | 속도 |
| --- | --- | --- | ---: | ---: |
| head | ZED-M `11295797` | `/zed/zed_node/left/image_rect_color/compressed` | 672x376 | 약 15 Hz |
| left wrist | D405 `335122270624` | `/camera_left/camera_left/color/image_rect_raw/compressed` | 640x480 | 약 15 Hz |
| right wrist | D405 `335122272052` | `/camera_right/camera_right/color/image_rect_raw/compressed` | 640x480 | 약 15 Hz |

Head는 ZED-M의 native VGA 영상이다. 별도
`/head_camera/...` crop/resize publisher는 사용하지 않는다. Wrist 영상의
방향 정규화가 필요하면 consumer인 Cyclo Intelligence에서 한 번만 적용한다.

추론 측이 읽는 로봇 상태는 `/joint_states`와 `/odom`이다. 추론 측 명령
출력은 다음 topic으로 돌아온다.

- left arm and gripper:
  `/leader/joint_trajectory_command_broadcaster_left/joint_trajectory`
- right arm and gripper:
  `/leader/joint_trajectory_command_broadcaster_right/joint_trajectory`
- head: `/leader/joystick_controller_left/joint_trajectory`
- lift: `/leader/joystick_controller_right/joint_trajectory`
- complete mobile base: `/cmd_vel` (`linear_x`, `linear_y`, `angular_z`)

Base action은 세 축 전체가 기본 계약이다. 특정 task나 한 축만을 가정하는
projection은 `ai_worker`에 넣지 않는다.

## 설정 소유권

| 파일 | 책임 |
| --- | --- |
| `ffw_bringup/config/common/zedm.yaml` | ZED-M native VGA, 15 Hz 획득·발행 |
| `ffw_bringup/launch/camera_realsense.launch.py` | 두 D405의 640x480@15 stream profile |
| `ffw_bringup/config/ffw_sg2_rev1_follower/rs_serial_1050.yaml` | 1050의 left/right camera serial mapping |
| `ffw_bringup/launch/ffw_sg2_follower_ai.launch.py` | SG2 controllers, cameras and lidar 조립 |
| Cyclo Intelligence robot config | observation 이름·순서, policy shape, wrist rotation, action mapping |

`rs_serial.yaml`의 값은 알고리즘 상수가 아니라 실제 장치 배치다. 다른 SG2에
배포할 때는 장치 시리얼과 left/right 역할을 다시 확인해야 한다.

## 1050 bringup과 검증

1050의 custom `ffw-autobringup` service는 `/run/robot_type`을 `sg2`로
설정하고 다음 launch를 선택한다.

```bash
export FFW_RS_SERIAL_PATH="$(ros2 pkg prefix --share ffw_bringup)/config/ffw_sg2_rev1_follower/rs_serial_1050.yaml"
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py \
  start_rviz:=false init_position:=false
```

`FFW_RS_SERIAL_PATH`를 지정하지 않으면 기존 common fallback을 사용한다. 따라서
1050 장치 시리얼이 다른 SG2의 기본값으로 새어 나가지 않는다. 배포 서비스도
launch 전에 같은 환경변수를 제공해야 한다.

실제 로봇에서 controller를 시작하거나 topic을 publish하는 검증은 E-stop,
작업 공간, 담당자 승인을 먼저 확인해야 한다. 다음은 bringup이 이미 실행 중일
때 사용하는 읽기 전용 확인 예시다.

```bash
ros2 topic type /zed/zed_node/left/image_rect_color/compressed
ros2 topic type /camera_left/camera_left/color/image_rect_raw/compressed
ros2 topic type /camera_right/camera_right/color/image_rect_raw/compressed
ros2 topic hz /zed/zed_node/left/image_rect_color/compressed
ros2 topic hz /camera_left/camera_left/color/image_rect_raw/compressed
ros2 topic hz /camera_right/camera_right/color/image_rect_raw/compressed
```

배포 후에는 세 image topic의 실제 크기와 속도, left/right 물리 방향,
`/joint_states`의 joint 이름, Cyclo Intelligence가 사용하는 command topic을
함께 확인한다. 단순히 launch가 종료되지 않았다는 사실만으로 검증 완료로
판정하지 않는다.

## 1050 host의 후속 정리 항목

2026-09-11 현재 host의
`docker/workspace/custom-services.d/ffw-autobringup/run`은 사용되지 않는
`head_camera_remap` executable의 존재를 build 완료 sentinel로 사용한다.
이 base는 remap executable을 제공하지 않으므로, 실제 배포 시 sentinel을
`ffw_bringup` package 자체의 설치 marker로 교체해야 한다. 이 파일은
배포 host의 untracked 운영 설정이며 이 저장소 브랜치에서 자동으로
덮어쓰지 않는다.

같은 host checkout의 speed limit, deadband, torque service 변경은 별도
안전 검증 대상이다. 카메라 계약 정리와 한 커밋에 섞지 않는다.
