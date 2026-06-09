# 🧹 Home Clean Mobile Robot

Nav2 기반 자율 청소 로봇 프로젝트 — ROS2 Humble + Gazebo 시뮬레이션

## 개요

TurtleBot3 Waffle 모델을 사용하여 Gazebo 가상 환경에서:
1. **SLAM 매핑** — 실내 환경을 자율적으로 탐색하며 지도를 생성
2. **맵 저장** — 생성된 Occupancy Grid 맵을 파일로 저장
3. **자율 주행** — 저장된 맵 기반으로 AMCL 위치 추정 + Nav2 네비게이션
4. **Coverage 청소** — opennav_coverage 서버로 완전 커버리지 경로를 계획하고 청소 수행

## 기술 스택

| 항목 | 패키지 | 관리 방식 |
|------|--------|-----------|
| 네비게이션 | [Navigation2](https://github.com/ros-planning/navigation2) (`humble`) | 소스 (vcstool) |
| SLAM | [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) (`humble`) | 소스 (vcstool) |
| Coverage Planning | [opennav_coverage](https://github.com/open-navigation/opennav_coverage) (`v1.2.1`) | 소스 (vcstool) |
| Coverage Library | [Fields2Cover](https://github.com/Fields2Cover/Fields2Cover) (`v1.2.1`) | 소스 (vcstool) |
| 로봇 플랫폼 | TurtleBot3 Waffle (`humble-devel`) | 소스 (vcstool) |
| 시뮬레이션 | Gazebo Classic | apt |
| 기타 유틸 | Eigen, PCL, OpenCV 등 | apt |

## 빠른 시작

```bash
# 1. 시스템 의존성 설치
cd ~/hyjoe_repositories/src/home-clean-mobile-robot
chmod +x install_deps.sh
./install_deps.sh

# 2. 소스 코드 다운로드 (vcstool)
vcs import . < deps.repos

# 3. rosdep 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# 4. 빌드
colcon build --symlink-install
source install/setup.bash

# 5. 환경 변수 설정
export TURTLEBOT3_MODEL=waffle

# 6. SLAM 매핑 실행
ros2 launch clean_robot_pkg 01_gazebo_slam.launch.py

# 7. (다른 터미널) 텔레옵으로 매핑
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 8. (다른 터미널) 맵 저장
ros2 launch clean_robot_pkg 02_save_map.launch.py

# 9. Coverage 청소
ros2 launch clean_robot_pkg 04_coverage_clean.launch.py
```

상세 가이드는 [docs/user_guide.md](docs/user_guide.md) 를 참고하세요.

## 프로젝트 구조

```
home-clean-mobile-robot/
├── deps.repos                    # vcstool 소스 관리
├── install_deps.sh               # 시스템 의존성 설치
├── src/
│   ├── (external sources)        # vcs import 소스 (gitignore)
│   └── clean_robot_pkg/          # 커스텀 패키지
│       ├── config/               # 파라미터 설정
│       ├── launch/               # Launch 파일 (01~04)
│       ├── maps/                 # 저장된 맵 파일
│       ├── scripts/              # Python 노드
│       └── rviz/                 # RViz 설정
└── docs/
    └── user_guide.md             # 실행 가이드 (한국어)
```

## 라이선스

이 프로젝트는 각 오픈소스 구성 요소의 원본 라이선스를 따릅니다.
