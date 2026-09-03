# FFW 1050 AprilTag 헤드 보정 상태

- 대상: `ffw-SNPR48A1050` (`ssh 1050`)
- 점검일: 2026-08-24 (KST)
- ROS 환경: `ai_worker` Docker 컨테이너, ROS 2 Jazzy, `ROS_DOMAIN_ID=30`
- 대상 로봇/설정: SG2 follower, `ffw_sg2_rev1_follower`

## 결론

보정값 측정·저장과 SG2 ros2_control xacro 수정은 완료됐다. 현재 저장된 Homing Offset은 다음과 같다.

| Joint | Dynamixel | 새 Homing Offset |
|---|---:|---:|
| `head_joint1` | 61 | -1 pulse |
| `head_joint2` | 62 | 29 pulses |

점검 당시 실행 중인 `ros2_control`은 xacro 수정 전에 시작됐으므로 새 값이 런타임에 아직 로드되지 않았다. 로봇을 재부팅하면 `ai_worker` 컨테이너가 자동 재시작되고 SG2 follower를 자동 브링업하면서 수정된 xacro를 다시 읽으므로, 새 offset이 반영된 컨트롤러로 기동될 것으로 확인됐다.

## 이전 결과와 비교

비교 기준은 `ffw_april_head_calibration` 저장소의 기존 커밋에 들어 있던 YAML(2026-04-29 18:25:57 KST)이다. 새 결과는 2026-08-24 14:49:35 KST에 저장됐다.

| 항목 | 이전 | 현재 | 변화량 |
|---|---:|---:|---:|
| `head_joint1` reference | 257 | 257 | 0 |
| `head_joint1` current | 257 | 256 | -1 |
| `head_joint1` delta | 0 | -1 | -1 |
| `head_joint2` reference | -20 | -20 | 0 |
| `head_joint2` current | -8 | 9 | +17 |
| `head_joint2` delta | 12 | 29 | +17 |

`pulse_resolution`은 이전과 현재 모두 2048 pulses/revolution이다. 각도 환산 시 현재 offset은 대략 `head_joint1=-0.176°`, `head_joint2=+5.098°`이며, 이전 대비 변화는 각각 `-0.176°`, `+2.988°`다.

## 확인 근거

1. 결과 YAML

   - 경로: `/home/robotis/ai_worker/ffw_april_head_calibration/config/ffw_head_joint_pulse_delta.yaml`
   - 저장 시각: 2026-08-24 14:49:35 KST
   - 저장값: `delta_pulse.head_joint1=-1`, `delta_pulse.head_joint2=29`

2. 보정 노드 로그

   - 보정 launch 시작: 2026-08-24 14:48:22 KST
   - 로그에서 `head_joint1=256 (delta=-1)`, `head_joint2=9 (delta=29)`가 반복적으로 안정되게 관측됐다.

3. 적용 대상 xacro

   - 경로: `/home/robotis/ai_worker/ffw_description/ros2_control/ffw_sg2_rev1_follower/ffw_sg2_follower.ros2_control.xacro`
   - `dxl61`: `<param name="Homing Offset">-1</param>`
   - `dxl62`: `<param name="Homing Offset">29</param>`
   - 저장소 HEAD에는 두 항목이 없었으며, 현재 작업 트리에서 새로 추가된 상태다.

4. 점검 당시 런타임 상태

   - `ros2_control_node` 시작: 2026-08-24 14:24:21 KST
   - xacro 수정: 2026-08-24 14:49:35 KST
   - 실행 중 `/robot_state_publisher`의 `robot_description`에서 `dxl61/62`에 `Homing Offset`이 없었다.
   - 따라서 점검 당시 런타임 컨트롤러는 보정 전 설정이다.
   - Dynamixel 읽기 서비스로 레지스터 직접 조회도 시도했으나 ID 61/62 모두 `result=False`여서 레지스터 값 자체는 이 방법으로 확인하지 못했다.

5. 재부팅 후 반영 경로

   - `ai_worker` 컨테이너 restart policy: `always`
   - 자동 서비스가 `/run/robot_type`을 `sg2`로 설정하고 `ffw_sg2_follower_ai.launch.py`를 실행한다.
   - 해당 launch의 기본 model은 `ffw_sg2_rev1_follower`다.
   - SG2 URDF는 `ros2_control/ffw_sg2_rev1_follower/ffw_sg2_follower.ros2_control.xacro`를 include한다.
   - install 영역의 xacro는 위 source 파일을 가리키는 symlink라 별도 rebuild 없이 수정 내용이 보인다.
   - 호스트의 source 파일은 bind mount에 있으므로 일반적인 로봇 재부팅 후에도 변경이 유지된다.

## 재부팅 후 검증

재부팅 후 다음 명령으로 런타임 `robot_description`에 두 offset이 들어갔는지 확인한다.

```bash
ssh 1050
docker exec -it ai_worker bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 param get /robot_state_publisher robot_description \
  | grep -o '<param name="Homing Offset">[-0-9]*</param>'
```

예상 출력:

```text
<param name="Homing Offset">-1</param>
<param name="Homing Offset">29</param>
```

추가로 `ros2_control_node` 시작 시각이 재부팅 이후인지 확인하고, 헤드의 물리적 중립 자세를 최종 확인해야 한다. 이 마지막 물리 자세 확인까지 통과해야 실제 로봇 기준 검증이 완료된 것으로 본다.
