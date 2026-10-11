# tmc_wrs_gazebo_worlds を Isaac 用メッセージビルドに通したときの問題と、上流への反映案

対象: https://github.com/ry0hei-kobayashi/tmc_wrs_gazebo ブランチ `ignition/humble`
（確認時 HEAD: `d7b5834` 2025-09-15 "[Mod] Compatible with ignition (models/ycb_*)"）
パッケージ: `tmc_wrs_gazebo_worlds`

hsr-omniverse2 側の記録日: 2026-10-11。別リポジトリ（tmc_wrs_gazebo）へ後で反映するためのメモ。

## 何が起きたか

`env_docker/Dockerfile.isaacsim` の msgbuild ステージ（当時 `ros:humble-ros-base-jammy`、現在は `ros:jazzy-ros-base` に
python3.11 を足した最小環境）で、Isaac 側が使う純 Python モジュール
`tmc_wrs_gazebo_worlds.randomizer` を得るために colcon でこのパッケージを建てたところ、
CMake の configure で落ちた。

```
CMake Error at CMakeLists.txt:23 (find_package):
  By not providing "Findxacro.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "xacro", but
  CMake did not find one.
```

`CMakeLists.txt` が

```cmake
find_package(xacro REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
```

を要求するが、これらは **world ファイル生成（`*.world.xacro` → `*.world`）のためだけ**に
使われている。Python パッケージ（`randomizer.py` / `spawn_objects`）のインストール
`ament_python_install_package(${PROJECT_NAME})` 自体は xacro も tf2 も要らない。

Isaac Sim 5.1 のコンテナでは ROS を apt で入れられない（同梱 rclpy を使う）ので、
xacro / tf2 を揃えた環境を msgbuild ステージに用意するのは本末転倒だった。

## hsr-omniverse2 側でどう回避したか

colcon を通さず、ソースの Python パッケージディレクトリをそのままコピーしている
（`env_docker/build_ros_msgs_py311.sh`）:

```bash
cp -rf "$SRC/tmc_wrs_gazebo/tmc_wrs_gazebo_worlds/tmc_wrs_gazebo_worlds" "$OUT/rclpy/"
```

`randomizer.py` の依存は `numpy` と `scipy.spatial.distance.pdist` だけなので、Isaac の
Python に `scipy` を pip で入れて import が通る。world / model データ（`share/`）は
Isaac 側では使わない（`worlds/` を自前で持つ）ので配っていない。

## 上流（tmc_wrs_gazebo）への反映案

目的: 「Python 部分だけ欲しい」環境でも `colcon build --packages-select tmc_wrs_gazebo_worlds`
が通るようにする。どれか 1 つで足りる。推奨は A。

### A. world 生成を xacro が見つかったときだけ行う（最小変更）

`CMakeLists.txt`:

```cmake
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# world 生成にしか使わないので任意依存にする
find_package(xacro QUIET)
# tf2 / tf2_ros は CMake 側では未使用 (exec_depend は package.xml で宣言済み)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/spawn_objects DESTINATION lib/${PROJECT_NAME})

if(xacro_FOUND)
  # ... 既存の foreach(_xacro ...) ブロックをそのまま ...
else()
  message(WARNING "xacro not found: *.world.xacro are not generated (Python package only)")
endif()
```

`package.xml`:

```xml
<build_depend>ament_cmake</build_depend>
<build_depend>ament_cmake_python</build_depend>
<!-- world 生成用。無くても Python パッケージは建つ -->
<build_depend condition="$WRS_GENERATE_WORLDS == 1">xacro</build_depend>
<exec_depend>python3-numpy</exec_depend>
<exec_depend>python3-scipy</exec_depend>
```

（`condition` 属性が煩わしければ `<build_depend>xacro</build_depend>` のまま残し、
rosdep だけ `--skip-keys xacro` で逃げる運用でもよい。）

### B. Python 部分を別パッケージに切り出す

`tmc_wrs_gazebo_worlds_py`（`ament_python`、`randomizer.py` + `spawn_objects`）を作り、
`tmc_wrs_gazebo_worlds` は world / model データ専用にする。Isaac 側はこちらだけを
依存に取ればよい。変更量は増えるが、依存関係が最も素直になる。

### C. 何もしない（hsr-omniverse2 側のコピー運用を続ける）

動くが、`randomizer.py` が更新されたときに追従を忘れる。A か B を推奨。

## ついでに直したい点（同じファイル群）

- `package.xml` に `python3-numpy` / `python3-scipy` の `exec_depend` が無い。
  `randomizer.py` は両方を import している。
- `setup.py` は `package_dir={"": "src"}` だが、実際のパッケージは
  `tmc_wrs_gazebo_worlds/tmc_wrs_gazebo_worlds/`（`src/` 無し）にある。
  `ament_python_install_package` 経由では使われないので実害は無いが、
  `pip install -e .` すると壊れる。`package_dir` を削除するか、ディレクトリを `src/` に移す。
- `setup.py` の `version="1.0.0"` と `package.xml` の `<version>2.0.0</version>` が食い違っている。

## 確認手順（上流を直した後）

```bash
docker run --rm -it ros:humble-ros-base-jammy bash -lc '
  apt-get update && apt-get install -y git python3-colcon-common-extensions &&
  mkdir -p /ws/src && cd /ws/src &&
  git clone -b ignition/humble https://github.com/ry0hei-kobayashi/tmc_wrs_gazebo.git &&
  cd /ws && source /opt/ros/humble/setup.bash &&
  colcon build --packages-select tmc_wrs_gazebo_worlds &&
  source install/setup.bash && python3 -c "from tmc_wrs_gazebo_worlds import randomizer; print(randomizer.__file__)"'
```

xacro を入れずにこれが通れば A/B は完了。通ったら hsr-omniverse2 の
`env_docker/build_ros_msgs_py311.sh` の `cp -rf ... tmc_wrs_gazebo_worlds` を
`--packages-select` への追加に戻せる。
