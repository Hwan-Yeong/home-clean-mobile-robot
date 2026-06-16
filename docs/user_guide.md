# 🧹 Nav2 기반 청소 로봇 실행 가이드

> **환경**: ROS2 Humble · Ubuntu 22.04 (WSL2) · Gazebo Classic · TurtleBot3 Waffle

---

## 목차

1. [사전 준비](#1-사전-준비)
2. [VCS (vcstool) 사용법](#2-vcs-vcstool-사용법)
3. [환경 설치](#3-환경-설치)
4. [빌드](#4-빌드)
5. [Step 1: Gazebo + SLAM 매핑](#5-step-1-gazebo--slam-매핑)
6. [Step 2: 맵 저장](#6-step-2-맵-저장)
7. [Step 3: 네비게이션 (Nav2 + AMCL)](#7-step-3-네비게이션-nav2--amcl)
8. [Step 4: Coverage 청소 실행](#8-step-4-coverage-청소-실행)
9. [트러블슈팅 FAQ](#9-트러블슈팅-faq)

---

## 1. 사전 준비

### ROS2 Humble 설치 확인

```bash
# ROS2 Humble이 설치되어 있는지 확인
source /opt/ros/humble/setup.bash
ros2 --version
```

### 필수 환경 변수 (.bashrc에 추가 권장)

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models' >> ~/.bashrc
source ~/.bashrc
```

---

## 2. VCS (vcstool) 사용법

### vcstool이란?

`vcstool`은 ROS2 커뮤니티에서 사용하는 **다중 저장소 관리 도구**입니다.
하나의 `.repos` 파일로 여러 git 저장소를 한 번에 clone하고 관리할 수 있습니다.

### 핵심 명령어

| 명령어 | 설명 | 예시 |
|--------|------|------|
| `vcs import` | `.repos` 파일 기반으로 저장소를 clone | `vcs import . < deps.repos` |
| `vcs pull` | 모든 저장소의 최신 변경사항 pull | `vcs pull src` |
| `vcs status` | 모든 저장소의 git status 확인 | `vcs status src` |
| `vcs export` | 현재 상태를 `.repos` 파일로 내보내기 | `vcs export --exact src > snapshot.repos` |
| `vcs log` | 모든 저장소의 최근 커밋 로그 | `vcs log -n 3 src` |

### `.repos` 파일 구조 설명

```yaml
repositories:
  src/navigation2:          # clone될 경로 (워크스페이스 기준 상대 경로)
    type: git               # 버전 관리 시스템 타입
    url: https://github.com/ros-planning/navigation2.git  # 저장소 URL
    version: humble         # branch 이름 또는 tag 이름
```

### 자주 쓰는 워크플로우

```bash
# 1. 처음 세팅 시: 모든 소스 클론
cd ~/{workspace}
vcs import . < deps.repos

# 2. 모든 저장소를 최신으로 업데이트
vcs pull src

# 3. 현재 상태 확인 (수정된 파일이 있는지)
vcs status src

# 4. 현재 정확한 커밋 해시를 기록 (재현 가능한 스냅샷)
vcs export --exact src > deps_snapshot.repos

# 5. 특정 시점 스냅샷으로 복원
vcs import . < deps_snapshot.repos
```

> **💡 Tip**: `vcs export --exact`로 내보낸 파일은 커밋 해시가 기록되므로,
> 나중에 정확히 같은 상태로 워크스페이스를 복원할 수 있습니다.
> 이를 활용하면 "이 시점에서 빌드가 성공했다"는 것을 보장할 수 있습니다.

---

## 3. 환경 설치

### Step 3-1: 시스템 의존성 설치

```bash
cd ~/{workspace}
chmod +x install_deps.sh
./install_deps.sh
```

이 스크립트는 다음을 설치합니다:
- Gazebo 시뮬레이션 패키지
- Fields2Cover 빌드 의존성 (Eigen, GDAL, GEOS 등)
- Python 라이브러리 (OpenCV, NumPy, Shapely)
- vcstool, colcon

### Step 3-2: 소스 코드 다운로드

```bash
cd ~/{workspace}
vcs import . < deps.repos
```

> 이 과정에서 약 1~5분 정도 소요됩니다 (Nav2 저장소가 큽니다).

### Step 3-3: rosdep 의존성 설치

```bash
# rosdep 초기화 (처음 한 번만)
sudo rosdep init 2>/dev/null || true
rosdep update

# 워크스페이스 의존성 설치
cd ~/{workspace}
rosdep install --from-paths src --ignore-src -r -y
```

---

## 4. 빌드

### 전체 빌드

```bash
cd ~/{workspace}
colcon build --symlink-install
```

> ⚠️ **첫 빌드는 15~30분 이상 소요**될 수 있습니다 (Nav2 전체 빌드).
> 메모리가 부족하면 `colcon build --symlink-install --parallel-workers 2`로 병렬 수를 제한하세요.

### 커스텀 패키지만 빌드 (변경 후 빠른 빌드)

```bash
colcon build --packages-select clean_robot_pkg --symlink-install
```

### 빌드 후 환경 소싱

```bash
source install/setup.bash
```

> **💡 Tip**: 매번 수동으로 소싱하지 않으려면 `.bashrc`에 추가:
> ```bash
> echo 'source ~/{workspace}/install/setup.bash' >> ~/.bashrc
> ```

---

## 5. Step 1: Gazebo + SLAM 매핑

### 실행

**터미널 1 — Gazebo + SLAM + RViz 실행:**

```bash
export TURTLEBOT3_MODEL=waffle
source install/setup.bash
ros2 launch clean_robot_pkg 01_gazebo_slam.launch.py
```

**터미널 2 — 텔레옵 (키보드로 로봇 조종):**

```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 텔레옵 조작키

```
   u    i    o
   j    k    l
   m    ,    .

i/k : 전진/후진
j/l : 좌회전/우회전
u/o : 좌측 전진/우측 전진
, : 후진 좌회전
. : 후진 우회전
q/z : 속도 증가/감소
```

### 매핑 팁

- 로봇을 **천천히** 움직이세요 (속도 0.1~0.15 권장)
- 벽과 **가까이** 다가가면 더 정확한 맵이 나옵니다
- **같은 곳을 다시 지나가면** 루프 클로저가 발동합니다 (정확도 향상)
- RViz에서 맵이 실시간으로 그려지는 것을 확인하세요

---

## 6. Step 2: 맵 저장

SLAM 매핑이 끝나면 (01번 launch가 아직 실행 중인 상태에서):

**터미널 3 — 맵 저장:**

```bash
source install/setup.bash
ros2 launch clean_robot_pkg 02_save_map.launch.py
```

커스텀 이름으로 저장하려면:

```bash
ros2 launch clean_robot_pkg 02_save_map.launch.py map_name:=my_house_map
```

### 저장된 파일 확인

```bash
ls -la install/clean_robot_pkg/share/clean_robot_pkg/maps/
# house_map.pgm  — 맵 이미지 파일
# house_map.yaml — 맵 메타데이터 (해상도, 원점 등)
```

> 맵 저장이 완료되면, **01번 launch를 종료**해도 됩니다 (Ctrl+C).

---

## 7. Step 3: 네비게이션 (Nav2 + AMCL)

저장된 맵으로 Nav2 전체 스택을 실행합니다.

```bash
export TURTLEBOT3_MODEL=waffle
source install/setup.bash
ros2 launch clean_robot_pkg 03_navigation.launch.py
```

커스텀 맵 사용:

```bash
ros2 launch clean_robot_pkg 03_navigation.launch.py map:=/path/to/your_map.yaml
```

### 초기 위치 설정

1. RViz에서 **"2D Pose Estimate"** 버튼 클릭
2. 로봇의 현재 위치(Gazebo에서 보이는 위치)에 **클릭 & 드래그**하여 방향 지정
3. AMCL 파티클이 수렴하면 위치 추정 완료

### 자율 주행 테스트

1. RViz에서 **"2D Nav Goal"** 버튼 클릭
2. 목표 지점에 **클릭 & 드래그**하여 방향 지정
3. 로봇이 경로를 계획하고 자율 주행하는 것을 확인

---

## 8. Step 4: Coverage 청소 실행

### 자동 청소 (전체 free-space)

```bash
export TURTLEBOT3_MODEL=waffle
source install/setup.bash
ros2 launch clean_robot_pkg 04_coverage_clean.launch.py
```

이 launch는 다음을 자동으로 수행합니다:

1. **Gazebo + Nav2 전체 스택** 실행
2. **Coverage Server** 실행
3. **맵→폴리곤 변환** (free-space 자동 추출)
4. **Coverage 경로 계산** (boustrophedon/snake 패턴)
5. **FollowPath로 경로 추종** (실제 청소 동작)

> ⚠️ **사전 조건**: Step 2에서 **맵이 저장되어 있어야** 합니다.
> 맵 파일이 없으면 map_server가 에러를 발생합니다.

### RViz에서 확인 가능한 것들

- **파란색 폴리곤**: 청소 구역 (cleaning_zone)
- **주황색 경로**: coverage 경로
- **녹색 경로**: Nav2 global plan
- **로봇 모델**: 실시간 위치

### 청소 완료 시

터미널에 다음과 같은 메시지가 출력됩니다:

```
========================================
  🎉 CLEANING COMPLETE!
  Coverage path successfully followed.
========================================
```

---

## 9. 트러블슈팅 FAQ

### Q1. Gazebo가 실행되지 않거나 모델이 보이지 않습니다

```bash
# Gazebo 모델 경로 확인
echo $GAZEBO_MODEL_PATH

# TurtleBot3 모델 환경 변수 확인
echo $TURTLEBOT3_MODEL  # 반드시 'waffle'이어야 함

# Gazebo 수동 실행 테스트
gazebo --verbose
```

### Q2. colcon build 시 메모리 부족 에러

```bash
# 병렬 빌드 수 제한
colcon build --symlink-install --parallel-workers 1

# 또는 swap 메모리 추가 (WSL2)
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

### Q3. SLAM 매핑 중 맵이 이상하게 나옵니다

- 로봇을 **더 천천히** 움직이세요
- 같은 구간을 **여러 번 왕복**하세요 (루프 클로저 강화)
- `slam_params.yaml`에서 `resolution`을 0.05 → 0.03으로 줄이면 더 정밀한 맵을 얻을 수 있습니다

### Q4. Coverage 경로가 생성되지 않습니다

- `/cleaning_zone` 토픽이 퍼블리시되는지 확인:
  ```bash
  ros2 topic echo /cleaning_zone
  ```
- `map_to_polygon_node`의 로그 확인 — 컨투어가 너무 작으면 무시됩니다
- Coverage server가 active 상태인지 확인:
  ```bash
  ros2 lifecycle get /coverage_server
  ```

### Q5. FollowPath 중 로봇이 멈춥니다

- **장애물에 걸린 경우**: Nav2 recovery behavior (spin, backup)가 자동 실행됩니다
- `controller_server` 로그에서 에러 확인
- `nav2_params.yaml`에서 `xy_goal_tolerance`를 0.25 → 0.35로 늘려보세요

### Q6. Fields2Cover 빌드 에러

```bash
# 반드시 v1.2.1 태그인지 확인
cd src/Fields2Cover
git status
git log -1  # 커밋 해시 확인

# v2.0.0은 호환되지 않음!
# 확인 후 태그 리셋:
git checkout v1.2.1
```

### Q7. WSL2에서 Gazebo GUI가 표시되지 않습니다

```bash
# X11 포워딩 확인
echo $DISPLAY  # :0 또는 비슷한 값이어야 함

# WSL2 + WSLg 사용 시 (Windows 11):
# 별도 설정 없이 자동으로 GUI가 표시됩니다

# 수동 X11 서버 사용 시:
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
export LIBGL_ALWAYS_INDIRECT=0
```

---

## 프로젝트 디렉토리 구조

```
home-clean-mobile-robot/
├── deps.repos                    # vcstool 소스 관리 (핵심!)
├── install_deps.sh               # apt 의존성 설치 스크립트
├── README.md                     # 프로젝트 개요
├── src/
│   ├── navigation2/              # [vcs] Nav2 (humble)
│   ├── slam_toolbox/             # [vcs] SLAM Toolbox (humble)
│   ├── Fields2Cover/             # [vcs] Coverage 알고리즘 (v1.2.1)
│   ├── opennav_coverage/         # [vcs] Coverage Server (v1.2.1)
│   ├── turtlebot3/               # [vcs] TurtleBot3 (humble-devel)
│   ├── turtlebot3_msgs/          # [vcs]
│   ├── turtlebot3_simulations/   # [vcs]
│   ├── DynamixelSDK/             # [vcs]
│   └── clean_robot_pkg/          # ✨ 커스텀 패키지
│       ├── config/
│       │   ├── nav2_params.yaml
│       │   ├── slam_params.yaml
│       │   └── coverage_params.yaml
│       ├── launch/
│       │   ├── 01_gazebo_slam.launch.py
│       │   ├── 02_save_map.launch.py
│       │   ├── 03_navigation.launch.py
│       │   └── 04_coverage_clean.launch.py
│       ├── maps/                 # SLAM 저장 맵
│       ├── scripts/
│       │   ├── map_to_polygon_node.py
│       │   └── coverage_client_node.py
│       └── rviz/
│           └── clean_robot.rviz
└── docs/
    └── user_guide.md             # ← 이 문서
```
