#!/usr/bin/env bash
# =============================================================================
#  install.sh — nav_1 一键部署（单文件，无其它脚本依赖）
#
#  流程：
#    1. 检查依赖            列出缺失的 apt / Python / 第三方库
#    2. 安装缺失依赖        先确认，再安装
#    3. 获取雷达和主机 IP   并行发现雷达；询问是否经 SDK 修改雷达 IP；
#                           主机地址可自动分配或手动输入；Wi-Fi 冲突时可迁移
#    4. 写入雷达配置        先确认，再改 Livox 配置（.bak 备份）；按型号选择驱动 launch
#    5. 获取底盘串口        自动发现 /dev/ttyACM*、/dev/ttyUSB*
#    6. 编写串口 udev 规则  先确认，再写规则并立即生效
#    7. 编译工作空间        colcon build（11 个包）
#    8. 配置 DDS（可选）    询问后选择 fastdds / cyclonedds 并持久化
#    9. 输出结果            汇总
#
#  用法见 ./install.sh --help
# =============================================================================

# 本脚本使用 bash 语法：从 zsh / sh 直接调用（如 `zsh install.sh`）时，
# 自动改用 bash 重新执行；被 source 时只提示，不替换当前 shell。
if [ -z "${BASH_VERSION:-}" ]; then
  case "${ZSH_EVAL_CONTEXT:-}" in
    *:file*)
      printf '这是 bash 脚本，请直接运行：bash %s\n' "$0" >&2
      return 1 ;;
  esac
  if [ -f "$0" ]; then
    exec bash "$0" "$@"
  fi
  printf '这是 bash 脚本，请用 bash 运行：bash install.sh\n' >&2
  return 1 2>/dev/null || exit 1
fi

set -Eeuo pipefail

SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname -- "$SCRIPT_PATH")"
SCRIPT_NAME="$(basename -- "$SCRIPT_PATH")"
SCRIPT_VERSION="2026-11-12.1"
WS="$SCRIPT_DIR"
MIGRATION_STATE="$WS/.lidar-network-migration.json"
# MID-360 与 MID-360S 使用不同的 Livox 驱动 launch（msg_MID360_launch.py /
# msg_MID360s_launch.py），两者读取的配置块也不同。识别出型号后改写这两个
# launch 里的驱动引用，避免 360 被 360S 的 launch 按 360S 初始化而收不到点云。
LIDAR_DRIVER_LAUNCH_FILES=(
  "$WS/src/bw_fastlio_nav2/launch/mid360_nav2_localization.launch.py"
  "$WS/src/bw_fastlio_nav2/launch/mid360_pgo_mapping.launch.py"
)

# ----------------------------------------------------------------------------
# 依赖清单（由 src/ 下 11 个 package.xml + launch/脚本实扫汇总）
# ----------------------------------------------------------------------------
BASE_APT=(
  build-essential cmake git curl wget unzip pkg-config ca-certificates lsb-release
  iproute2 iputils-arping
  python3-pip python3-dev python3-setuptools python3-wheel python3-colcon-common-extensions
  pybind11-dev libeigen3-dev libpcl-dev libboost-all-dev libyaml-cpp-dev
  libapr1-dev libaprutil1-dev libopencv-dev python3-pyqt5
)
ROS_APT=(
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-common
  ros-humble-nav2-map-server ros-humble-nav2-costmap-2d ros-humble-nav2-collision-monitor
  ros-humble-nav2-lifecycle-manager ros-humble-nav2-mppi-controller
  ros-humble-nav2-smac-planner ros-humble-nav2-util ros-humble-nav2-bt-navigator
  ros-humble-bondcpp ros-humble-serial-driver ros-humble-spatio-temporal-voxel-layer
  ros-humble-gtsam ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-cv-bridge
  ros-humble-tf2-eigen ros-humble-tf2-sensor-msgs ros-humble-tf2-geometry-msgs
  ros-humble-tf2-tools ros-humble-rviz2 ros-humble-robot-state-publisher
  ros-humble-joint-state-publisher ros-humble-xacro ros-humble-urdf
  ros-humble-ros2launch ros-humble-rqt-gui ros-humble-rqt-gui-py
  ros-humble-python-qt-binding ros-humble-pybind11-vendor ros-humble-rosbag2
  ros-humble-rosbag2-py ros-humble-ros-testing ros-humble-octomap-server
)

PY_MODULES=(numpy scipy yaml PIL matplotlib rosbags PyQt5 pytest)
declare -A PY_PIP_NAME=(
  [numpy]="numpy==1.24.4" [scipy]="scipy" [yaml]="PyYAML" [PIL]="Pillow"
  [matplotlib]="matplotlib" [rosbags]="rosbags" [pytest]="pytest"
)

BUILD_PACKAGES=(
  bw_interface bw_utils bw_bringup bw_serial fast_lio fastlio_lc_pgo ndt_omp_ros2
  lidar_localization_ros2 livox_ros_driver2 bw_fastlio_nav2 bw_std_description
)

LIVOX_SDK_REF="08f523c"
LIVOX_STATIC_LIB="/usr/local/lib/liblivox_lidar_sdk_static.a"
LIVOX_HEADER="/usr/local/include/livox_lidar_api.h"
PIP_INDEX="https://pypi.tuna.tsinghua.edu.cn/simple"
PIP_INDEX_FALLBACK="https://pypi.org/simple"
SUBNET=""                              # 当前雷达 /24 前缀，迁移后同步更新
LIDAR_UDP_PORT="${LIDAR_UDP_PORT:-56000}"  # Livox-SDK2 雷达发现端口
LINK_NAME="bw_chassis"
UDEV_RULE="/etc/udev/rules.d/99-bw-nav-serial.rules"

# ----------------------------------------------------------------------------
# 运行参数 / 状态
# ----------------------------------------------------------------------------
ASSUME_YES=false
DRY_RUN=false
CHECK_ONLY=false
JOBS="$(nproc 2>/dev/null || echo 4)"
# lidar_localization 组件每个 TU 的编译峰值约 2.9 GiB（PCL/rclcpp/Eigen 模板
# 在 -O3 下全部展开，-O 级别几乎不影响），最重的 component_subscribers.cpp
# 约 4.5 GiB。colcon 在环境无 MAKEFLAGS 时会自动给 make 追加 -j$(nproc)；
# 15 GiB 的 NUC 上 16 个重型 TU 同时展开必然耗尽内存并整机卡死。
JOBS_MEM_TU_TYPICAL_MIB=3072
JOBS_MEM_TU_WORST_MIB=4608
JOBS_MEM_RESERVE_MIB=2048
HOST_IP_ARG=""
LIDAR_IP_ARG=""
LIDAR_MODEL_ARG=""
SERIAL_ARG=""

HOST_IP=""
HOST_CIDR=""
HOST_IFACE=""
HOST_IP_MODE=""
WIRELESS_IGNORED=()
LOCAL_IPS=()          # 本机所有 IPv4，用来把“自己”从雷达候选里剔掉
LIDAR_IP=""
LIDAR_MODEL=""
LIDAR_DEV_TYPE=""
LIDAR_SN=""
LIDAR_IFACE=""
LIDAR_IP_CHANGED=false   # 用户在本轮显式改过雷达 IP：不再自动迁移网段
SERIAL_DEV=""
SERIAL_VID=""
SERIAL_PID=""
SERIAL_NO=""
SERIAL_MODEL=""
SERIAL_PORT_PATH=""

LOG_FILE=""
SUDO_READY=false
SUDO_KEEPALIVE_PID=""

MISSING_APT=()
MISSING_PIP=()
NEED_LIVOX=false
NEED_GTSAM=false
APT_INSTALLED=0
PIP_INSTALLED=0
CONFIG_CHANGED=()
UDEV_WRITTEN=false
BUILD_RESULT="未执行"
BUILD_FAILED_PKGS=""
ERRORS=()
SKIPPED=()

# ----------------------------------------------------------------------------
# 输出
# ----------------------------------------------------------------------------
if [[ -t 1 ]]; then
  C_RED=$'\033[31m'; C_GRN=$'\033[32m'; C_YEL=$'\033[33m'; C_CYA=$'\033[36m'
  C_DIM=$'\033[2m'; C_BLD=$'\033[1m'; C_RST=$'\033[0m'
else
  C_RED=""; C_GRN=""; C_YEL=""; C_CYA=""; C_DIM=""; C_BLD=""; C_RST=""
fi

ts() { date +%H:%M:%S; }
hr() { printf '%s\n' "${C_DIM}────────────────────────────────────────────────────────────${C_RST}"; }
act() { printf '%s\n' "[$(ts)] ${C_CYA}→${C_RST} $*"; }
ok() { printf '%s\n' "[$(ts)] ${C_GRN}✓${C_RST} $*"; }
warn() { printf '%s\n' "[$(ts)] ${C_YEL}!${C_RST} $*"; }
err() { printf '%s\n' "[$(ts)] ${C_RED}✗${C_RST} $*" >&2; }
skip() { printf '%s\n' "[$(ts)] ${C_DIM}↷${C_RST} $*"; }
info() { printf '%s\n' "[$(ts)] ${C_DIM}·${C_RST} $*"; }

step() {
  printf '\n'
  hr
  printf '%s\n' "[$(ts)] ${C_BLD}步骤 $1/$2  $3${C_RST}"
  hr
}

usage() {
  cat <<EOF
${SCRIPT_NAME} — nav_1 一键部署（单文件）

流程：检查依赖 → 安装缺失依赖 → 获取雷达和主机 IP（发现后可选择经 SDK
      修改雷达 IP）→ 写入雷达配置（含按型号切换 MID-360 / MID-360S 驱动
      launch）→ 获取串口 → 编写 udev 规则 → 编译 → 配置 DDS（可选）→ 输出结果

用法：
  ./${SCRIPT_NAME} [选项]

选项：
  -y, --yes             所有确认都取默认值（雷达 IP 修改、DDS 配置取“不修改”）
      --check           只检查依赖，不安装、不写配置、不编译
      --dry-run         只打印将要执行的动作，不改系统
      --host-ip <ip>    主机在雷达网段的 IP（不填则现场探测或询问）
      --lidar-ip <ip>   雷达 IP（不填则现场发现或询问，网段由它推导）
      --lidar-model <m> 雷达型号 MID360 / MID360S（不填则自动探测；探测不到时用）
      --serial <dev>    直接指定底盘串口，如 /dev/ttyACM0
      --link-name <n>   udev 固定串口名（默认 ${LINK_NAME}）
      --jobs <n>        编译并行度上限（默认 CPU 核数：${JOBS}，按可用内存自动下调）
  -h, --help            显示本帮助

网络：各有线接口并行广播发现，不限制雷达 IPv4 网段（不跨路由器）。
      侦测到雷达 IP 后会询问是否修改：选择“是”需输入目标 IP；同网段直接
      经 SDK 改写并验证，跨网段会一并迁移有线主机地址并持久化。
      雷达网段与任一无线网卡网段相同时，询问是否把雷达和有线主机一起迁移到
      未占用的私有 /24 网段（优先 192.168.10.x）；有线连接通过 NetworkManager
      持久化，无线保持不变。
      请先停止 Livox 驱动；--dry-run 只预览，不向雷达写入命令。

示例：
  ./${SCRIPT_NAME}                                   # 正常部署，逐步确认
  ./${SCRIPT_NAME} --check                           # 只体检
  ./${SCRIPT_NAME} -y --dry-run                      # 演练
  ./${SCRIPT_NAME} --lidar-ip <雷达IP> --host-ip <本机在雷达网段的IP>
EOF
}

parse_args() {
  while (( $# > 0 )); do
    case "$1" in
      -y|--yes) ASSUME_YES=true; shift ;;
      --check) CHECK_ONLY=true; shift ;;
      --dry-run) DRY_RUN=true; shift ;;
      --host-ip) HOST_IP_ARG="${2:?--host-ip 需要 IP}"; shift 2 ;;
      --lidar-ip) LIDAR_IP_ARG="${2:?--lidar-ip 需要 IP}"; shift 2 ;;
      --lidar-model) LIDAR_MODEL_ARG="${2:?--lidar-model 需要 MID360 或 MID360S}"; shift 2 ;;
      --serial) SERIAL_ARG="${2:?--serial 需要设备路径}"; shift 2 ;;
      --link-name) LINK_NAME="${2:?--link-name 需要名称}"; shift 2 ;;
      --jobs) JOBS="${2:?--jobs 需要数字}"; shift 2 ;;
      -h|--help) usage; exit 0 ;;
      *) err "未知参数：$1（--help 查看用法）"; exit 2 ;;
    esac
  done
  [[ $JOBS =~ ^[1-9][0-9]*$ ]] || { err "--jobs 需要正整数：$JOBS"; exit 2; }
  if [[ -n $LIDAR_MODEL_ARG ]]; then
    if ! LIDAR_MODEL="$(normalize_lidar_model "$LIDAR_MODEL_ARG")"; then
      err "无法识别 --lidar-model：${LIDAR_MODEL_ARG}（可选 MID360 / MID360S）"
      exit 2
    fi
  fi
}

# 按当前可用内存给出安全的 make 并行度，--jobs 作为上限。
# 峰值模型：最重的一个 TU + 其余 (n-1) 个普通 TU 必须能同时装进可用内存。
safe_make_jobs() {
  local jobs="$JOBS" avail_mib=0 mem_jobs
  avail_mib=$(awk '/MemAvailable:/ {print int($2 / 1024)}' /proc/meminfo 2>/dev/null || echo 0)
  if (( avail_mib < JOBS_MEM_RESERVE_MIB + JOBS_MEM_TU_WORST_MIB )); then
    mem_jobs=1
  else
    mem_jobs=$(( 1 + (avail_mib - JOBS_MEM_RESERVE_MIB - JOBS_MEM_TU_WORST_MIB) / JOBS_MEM_TU_TYPICAL_MIB ))
  fi
  if (( mem_jobs < jobs )); then
    jobs=$mem_jobs
  fi
  printf '%s' "$jobs"
}

# ----------------------------------------------------------------------------
# 交互
# ----------------------------------------------------------------------------
ask() { # ask "提示" [默认值] -> stdout
  local prompt="$1" def="${2-}" ans=""
  if $ASSUME_YES || [[ ! -t 0 ]]; then
    printf '%s' "$def"
    return 0
  fi
  if [[ -w /dev/tty && -r /dev/tty ]]; then
    printf '%s' "$prompt" > /dev/tty
    IFS= read -r ans < /dev/tty || ans=""
  else
    printf '%s' "$prompt"
    IFS= read -r ans || ans=""
  fi
  [[ -n $ans ]] || ans="$def"
  printf '%s' "$ans"
}

confirm() { # confirm "问题" [Y|N]
  local prompt="$1" def="${2:-Y}" ans
  if $ASSUME_YES; then
    printf '%s\n' "[$(ts)] ${C_DIM}?${C_RST} ${prompt} ${C_DIM}[自动回答 ${def}]${C_RST}" >&2
    [[ $def == [Yy]* ]]
    return
  fi
  ans="$(ask "${prompt} [$([[ $def == [Yy]* ]] && echo 'Y/n' || echo 'y/N')] " "$def")"
  [[ -n $ans ]] || ans="$def"
  [[ $ans == [Yy]* || $ans == 是 || $ans == 好 ]]
}

# ----------------------------------------------------------------------------
# sudo
# ----------------------------------------------------------------------------
cleanup() {
  if [[ -n $SUDO_KEEPALIVE_PID ]]; then
    kill "$SUDO_KEEPALIVE_PID" 2>/dev/null || true
  fi
}

ensure_sudo() {
  $DRY_RUN && return 0
  [[ $EUID -eq 0 ]] && return 0
  $SUDO_READY && return 0
  command -v sudo >/dev/null 2>&1 || { err "需要 root 或 sudo 才能继续"; exit 1; }
  info "需要管理员权限，请输入 sudo 密码（只问这一次）"
  sudo -v || { err "sudo 认证失败"; exit 1; }
  SUDO_READY=true
  ( while kill -0 "$$" 2>/dev/null; do sudo -n true 2>/dev/null || true; sleep 50; done ) &
  SUDO_KEEPALIVE_PID=$!
}

run_root() {
  if $DRY_RUN; then act "[dry-run] sudo $*"; return 0; fi
  if [[ $EUID -eq 0 ]]; then "$@"; else ensure_sudo; sudo "$@"; fi
}

run_root_shell() { # run_root_shell "命令字符串"（用于重定向/管道）
  if $DRY_RUN; then act "[dry-run] (root) $1"; return 0; fi
  if [[ $EUID -eq 0 ]]; then bash -c "$1"; else ensure_sudo; sudo bash -c "$1"; fi
}

# ----------------------------------------------------------------------------
# 小工具
# ----------------------------------------------------------------------------
pkg_installed() { dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q 'install ok installed'; }

py_ok() { python3 -c "import $1" >/dev/null 2>&1; }

numpy_ok() {
  python3 - <<'PY' >/dev/null 2>&1
import sys
try:
    import numpy as np
except Exception:
    sys.exit(1)
sys.exit(0 if np.__version__.split(".")[0] == "1" else 1)
PY
}

livox_ok() { [[ -f $LIVOX_STATIC_LIB && -f $LIVOX_HEADER ]]; }

gtsam_ok() {
  pkg_installed ros-humble-gtsam && return 0
  local d
  for d in /opt/ros/humble /usr/local; do
    [[ -n "$(find "$d" -name GTSAMConfig.cmake -print -quit 2>/dev/null)" ]] && return 0
  done
  return 1
}

valid_ipv4() {
  local ip="$1" o x
  [[ $ip =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || return 1
  IFS='.' read -r -a o <<<"$ip"
  for x in "${o[@]}"; do (( x >= 0 && x <= 255 )) || return 1; done
  return 0
}

on_subnet() { [[ -n ${SUBNET:-} && $1 == "${SUBNET}."* ]]; }

# ----------------------------------------------------------------------------
# 步骤 1：检查依赖
# ----------------------------------------------------------------------------
check_apt_list() {
  local p
  for p in "$@"; do
    pkg_installed "$p" || MISSING_APT+=("$p")
  done
}

check_python() {
  local m
  for m in "${PY_MODULES[@]}"; do
    case "$m" in
      numpy) numpy_ok || MISSING_PIP+=("numpy") ;;
      *) py_ok "$m" || MISSING_PIP+=("$m") ;;
    esac
  done
}

check_dependencies() {
  step 1 9 "检查依赖"

  act "检查系统与 ROS 2"
  info "系统：$( (. /etc/os-release 2>/dev/null && echo "${PRETTY_NAME:-unknown}") || uname -s)  内核：$(uname -r)  Python：$(python3 -V 2>&1)"
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    ok "ROS 2 Humble：/opt/ros/humble"
  else
    err "未找到 /opt/ros/humble/setup.bash，请先安装 ROS 2 Humble"
    exit 1
  fi
  if command -v colcon >/dev/null 2>&1; then ok "colcon：$(command -v colcon)"; else warn "未找到 colcon（安装步骤会补 python3-colcon-common-extensions）"; fi
  command -v python3 >/dev/null 2>&1 || { err "缺少 python3"; exit 1; }

  act "检查 apt 依赖（基础 ${#BASE_APT[@]} 个 + ROS ${#ROS_APT[@]} 个）"
  check_apt_list "${BASE_APT[@]}" "${ROS_APT[@]}"

  act "检查 Python 依赖（${#PY_MODULES[@]} 个模块）"
  check_python

  act "检查第三方库"
  if livox_ok; then
    ok "Livox-SDK2 已安装（liblivox_lidar_sdk_static.a + livox_lidar_api.h）"
  else
    NEED_LIVOX=true
    warn "Livox-SDK2 未安装或缺少静态库，livox_ros_driver2 会编译失败"
  fi
  if gtsam_ok; then
    ok "GTSAM 已安装"
  else
    NEED_GTSAM=true
    warn "GTSAM 未安装，fastlio_lc_pgo 无法编译"
  fi

  printf '\n'
  if (( ${#MISSING_APT[@]} == 0 )); then ok "apt 依赖：全部已安装"
  else warn "apt 依赖：缺 ${#MISSING_APT[@]} 个 → ${MISSING_APT[*]}"; fi
  if (( ${#MISSING_PIP[@]} == 0 )); then ok "Python 依赖：全部可用"
  else warn "Python 依赖：缺 ${#MISSING_PIP[@]} 个 → ${MISSING_PIP[*]}"; fi
  if (( ${#MISSING_APT[@]} + ${#MISSING_PIP[@]} == 0 )) && ! $NEED_LIVOX && ! $NEED_GTSAM; then
    ok "依赖检查通过"
  fi
}

# ----------------------------------------------------------------------------
# 步骤 2：安装缺失依赖
# ----------------------------------------------------------------------------
apt_install() {
  local -a pkgs=("$@")
  (( ${#pkgs[@]} )) || return 0
  if $DRY_RUN; then
    act "[dry-run] sudo apt-get install -y ${pkgs[*]}"
    return 0
  fi
  ensure_sudo
  info "apt-get update"
  run_root env DEBIAN_FRONTEND=noninteractive apt-get update -qq || warn "apt-get update 失败，继续用现有索引"
  info "apt-get install -y ${pkgs[*]}"
  if run_root env DEBIAN_FRONTEND=noninteractive apt-get install -y "${pkgs[@]}"; then
    ok "apt 安装完成：${#pkgs[@]} 个包"
  else
    err "批量安装失败，改为逐个安装定位问题包"
    local p failed=0
    for p in "${pkgs[@]}"; do
      if pkg_installed "$p"; then skip "已安装：$p"; continue; fi
      if run_root env DEBIAN_FRONTEND=noninteractive apt-get install -y "$p"; then
        ok "已安装：$p"
      else
        err "安装失败：$p"; failed=$((failed + 1))
      fi
    done
    (( failed == 0 )) || return 1
  fi
  local p missing=0
  for p in "${pkgs[@]}"; do pkg_installed "$p" || { err "仍未安装：$p"; missing=$((missing + 1)); }; done
  (( missing == 0 )) || return 1
  APT_INSTALLED=${#pkgs[@]}
}

pip_install() {
  local -a mods=("$@") pkgs=()
  (( ${#mods[@]} )) || return 0
  local m
  for m in "${mods[@]}"; do
    if [[ $m == PyQt5 ]]; then
      apt_install python3-pyqt5 || true
      continue
    fi
    pkgs+=("${PY_PIP_NAME[$m]:-$m}")
  done
  (( ${#pkgs[@]} )) || return 0
  if $DRY_RUN; then
    act "[dry-run] python3 -m pip install --user -i ${PIP_INDEX} ${pkgs[*]}"
    return 0
  fi
  python3 -m pip --version >/dev/null 2>&1 || { err "python3 -m pip 不可用"; return 1; }
  info "pip install --user -i ${PIP_INDEX} ${pkgs[*]}"
  if ! python3 -m pip install --user -i "$PIP_INDEX" "${pkgs[@]}"; then
    warn "清华源失败，回退官方源"
    python3 -m pip install --user -i "$PIP_INDEX_FALLBACK" "${pkgs[@]}" || return 1
  fi
  PIP_INSTALLED=${#pkgs[@]}
  ok "pip 安装完成：${#pkgs[@]} 个包"
}

install_livox_sdk() {
  $NEED_LIVOX || { skip "Livox-SDK2 已安装，跳过"; return 0; }
  act "源码安装 Livox-SDK2（提交 ${LIVOX_SDK_REF}）"
  if $DRY_RUN; then
    act "[dry-run] 下载 Livox-SDK2 ${LIVOX_SDK_REF} → cmake 构建 → sudo cmake --install"
    return 0
  fi
  local tmp
  tmp="$(mktemp -d /tmp/livox-sdk2.XXXXXX)"
  local -a urls=(
    "https://ghfast.top/https://github.com/Livox-SDK/Livox-SDK2/archive/${LIVOX_SDK_REF}.tar.gz"
    "https://gh-proxy.com/https://github.com/Livox-SDK/Livox-SDK2/archive/${LIVOX_SDK_REF}.tar.gz"
    "https://github.com/Livox-SDK/Livox-SDK2/archive/${LIVOX_SDK_REF}.tar.gz"
  )
  local url got=""
  for url in "${urls[@]}"; do
    info "尝试下载：$url"
    if curl -fL --retry 2 --connect-timeout 15 -o "$tmp/sdk.tar.gz" "$url"; then
      if tar -tzf "$tmp/sdk.tar.gz" >/dev/null 2>&1; then got="$url"; break; fi
      warn "下载内容不是有效压缩包：$url"
    fi
  done
  if [[ -z $got ]]; then
    rm -rf "$tmp"
    err "Livox-SDK2 下载失败（镜像都不通）"
    return 1
  fi
  ok "下载成功：$got"
  mkdir -p "$tmp/src"
  tar -xzf "$tmp/sdk.tar.gz" -C "$tmp/src" --strip-components=1
  cmake -S "$tmp/src" -B "$tmp/build" -DCMAKE_BUILD_TYPE=Release >/dev/null
  cmake --build "$tmp/build" -j"$JOBS" >/dev/null
  ensure_sudo
  run_root cmake --install "$tmp/build" >/dev/null
  run_root ldconfig || true
  rm -rf "$tmp"
  if livox_ok; then
    ok "Livox-SDK2 安装完成：${LIVOX_STATIC_LIB}"
    NEED_LIVOX=false
  else
    err "Livox-SDK2 安装后仍找不到静态库/头文件"
    return 1
  fi
}

install_missing() {
  step 2 9 "安装缺失依赖"
  if (( ${#MISSING_APT[@]} == 0 )) && (( ${#MISSING_PIP[@]} == 0 )) && ! $NEED_LIVOX; then
    ok "没有缺失依赖，跳过"
    return 0
  fi

  printf '\n'
  info "将要安装："
  (( ${#MISSING_APT[@]} )) && printf '    apt（%d 个）：%s\n' "${#MISSING_APT[@]}" "${MISSING_APT[*]}"
  (( ${#MISSING_PIP[@]} )) && printf '    pip（%d 个）：%s\n' "${#MISSING_PIP[@]}" "${MISSING_PIP[*]}"
  $NEED_LIVOX && printf '    源码：Livox-SDK2（提交 %s）\n' "$LIVOX_SDK_REF"
  printf '\n'

  if ! confirm "是否安装以上缺失依赖？" Y; then
    warn "已跳过依赖安装（后续编译可能失败）"
    SKIPPED+=("依赖安装（用户跳过）")
    return 0
  fi

  local rc=0
  (( ${#MISSING_APT[@]} )) && { apt_install "${MISSING_APT[@]}" || rc=1; }
  (( ${#MISSING_PIP[@]} )) && { pip_install "${MISSING_PIP[@]}" || rc=1; }
  install_livox_sdk || rc=1

  if (( rc == 0 )); then
    ok "缺失依赖处理完成"
  else
    err "部分依赖安装失败（见上方日志）"
    ERRORS+=("依赖安装未全部成功")
  fi
}

# ----------------------------------------------------------------------------
# 步骤 3：获取雷达和主机 IP
# ----------------------------------------------------------------------------
is_wired_iface() { # 只认有线以太网：有 device、无 wireless、type=1
  local iif="$1" dir="/sys/class/net/$1"
  [[ -n $iif ]] || return 1
  if [[ -d $dir ]]; then
    [[ -e "$dir/wireless" ]] && return 1
    [[ -e "$dir/device" ]] || return 1
    [[ "$(cat "$dir/type" 2>/dev/null || true)" == "1" ]] || return 1
    return 0
  fi
  # 没有 sysfs 时按命名兜底：en*/eth*
  [[ $iif =~ ^(en|eth) ]]
}

wired_ifaces() { # 注意：pipefail 下循环体必须总是返回 0，否则 sort 管道会整体失败
  local i
  if [[ -d /sys/class/net ]]; then
    for i in /sys/class/net/*; do
      i="$(basename "$i")"
      if is_wired_iface "$i"; then printf '%s\n' "$i"; fi
    done | sort
  else
    { ip -o link show 2>/dev/null | awk -F': ' '{print $2}' | grep -E '^(en|eth)' || true; } | sort -u
  fi
}

iface_state() {
  local st
  st="$(cat "/sys/class/net/$1/operstate" 2>/dev/null || true)"
  case "$st" in
    up) printf 'UP' ;;
    down) printf 'DOWN' ;;
    *) [[ -n $st ]] && printf '%s' "$st" || printf '?' ;;
  esac
}

# 本机所有 IPv4（用于剔除“自己”：广播回环/自 ping 都会命中本机地址）
local_ips() { ip -4 -o addr show 2>/dev/null | awk '$3 == "inet" { sub(/\/.*/, "", $4); print $4 }'; }

is_local_ip() { # <ip>：本机任意网卡的地址都算
  local want="$1" got
  for got in "${LOCAL_IPS[@]}"; do [[ $got == "$want" ]] && return 0; done
  return 1
}

find_host_net() { # 只在有线网卡上找 SUBNET.x；无线上的同网段地址记到 WIRELESS_IGNORED
  local ifc cidr
  WIRELESS_IGNORED=()
  HOST_IFACE=""; HOST_CIDR=""; HOST_IP=""
  while read -r ifc cidr; do
    [[ -n $ifc && -n $cidr ]] || continue
    if is_wired_iface "$ifc"; then
      [[ -n $LIDAR_IFACE && $ifc != "$LIDAR_IFACE" ]] && continue
      HOST_IFACE="$ifc"; HOST_CIDR="$cidr"; HOST_IP="${cidr%/*}"
      return 0
    fi
    if [[ -e "/sys/class/net/$ifc/wireless" ]]; then
      WIRELESS_IGNORED+=("$ifc $cidr")
    fi
  done < <(ip -4 -o addr show 2>/dev/null | awk -v s="$SUBNET." '$4 ~ ("^" s "[0-9]+/") {print $2, $4}')
  return 1
}

json_field() { # <file> <host|lidar>
  python3 - "$1" "$2" <<'PY' 2>/dev/null || true
import json, sys
path, what = sys.argv[1], sys.argv[2]
try:
    d = json.load(open(path))
except Exception:
    sys.exit(0)
if what == "lidar":
    for c in d.get("lidar_configs", []) or []:
        if isinstance(c, dict) and c.get("ip"):
            print(c["ip"]); break
else:
    for block in d.values():
        if not isinstance(block, dict):
            continue
        hni = block.get("host_net_info")
        if isinstance(hni, list):
            for item in hni:
                if isinstance(item, dict) and item.get("host_ip"):
                    print(item["host_ip"]); raise SystemExit
        elif isinstance(hni, dict):
            for k in ("cmd_data_ip", "point_data_ip", "imu_data_ip", "push_msg_ip"):
                if hni.get(k):
                    print(hni[k]); raise SystemExit
PY
}

livox_config_files() {
  local d f
  for d in "$WS/src/livox_ros_driver2/config" "$WS/install/livox_ros_driver2/share/livox_ros_driver2/config"; do
    [[ -d $d ]] || continue
    for f in MID360s_config.json MID360_config.json; do
      [[ -f "$d/$f" ]] && printf '%s\n' "$d/$f"
    done
  done
}

first_config() {
  local d
  for d in "$WS/src/livox_ros_driver2/config" "$WS/install/livox_ros_driver2/share/livox_ros_driver2/config"; do
    [[ -f "$d/MID360s_config.json" ]] && { printf '%s' "$d/MID360s_config.json"; return 0; }
  done
  return 1
}

ping_one() {
  local -a args=(-c1 -W1 -n)
  if [[ -n ${HOST_IFACE:-} && -n ${HOST_IP:-} ]] \
     && ip -4 -o addr show dev "$HOST_IFACE" 2>/dev/null | grep -q " ${HOST_IP}/"; then
    args+=(-I "$HOST_IFACE")
  fi
  ping "${args[@]}" "$1" >/dev/null 2>&1
}

ask_ipv4() { # ask_ipv4 "提示" <默认值> → 结果放 IP_ANSWER；最多问 3 次后取默认
  local prompt="$1" def="$2" tries=0 ans=""
  while (( tries < 3 )); do
    ans="$(ask "$prompt" "$def")"
    if valid_ipv4 "$ans" && on_subnet "$ans"; then
      IP_ANSWER="$ans"; return 0
    fi
    warn "IP 不合法或不在 ${SUBNET}.0/24：$ans"
    tries=$((tries + 1))
  done
  warn "连续 3 次输入无效，改用默认值 ${def}"
  IP_ANSWER="$def"
}

choose_wired_iface() { # 选一块有线网卡，结果放 CHOSEN_IFACE；没有则返回 1
  local -a ifaces=()
  mapfile -t ifaces < <(wired_ifaces)
  CHOSEN_IFACE=""
  if (( ${#ifaces[@]} == 0 )); then
    warn "没有找到有线网卡（要求：有 device、非 wireless、type=1）"
    return 1
  fi
  if (( ${#ifaces[@]} == 1 )); then
    CHOSEN_IFACE="${ifaces[0]}"
    info "唯一有线网卡：${CHOSEN_IFACE}（$(iface_state "$CHOSEN_IFACE")）"
    return 0
  fi
  info "可选有线网卡："
  local i n=1
  for i in "${ifaces[@]}"; do
    printf '    %d) %s（%s）\n' "$n" "$i" "$(iface_state "$i")"
    n=$((n + 1))
  done
  local pick
  pick="$(ask "请选择连接 MID360 的网卡 [1]: " "1")"
  [[ $pick =~ ^[0-9]+$ ]] && (( pick >= 1 && pick <= ${#ifaces[@]} )) || pick=1
  CHOSEN_IFACE="${ifaces[$((pick - 1))]}"
}

apply_host_address() { # <iface> <ip>：确认后把 ip/24 加到网卡（可选 nmcli 持久化）
  local iface="$1" ip="$2"
  HOST_IFACE="$iface"; HOST_CIDR="${ip}/24"; HOST_IP="$ip"
  if ip -4 -o addr show dev "$iface" 2>/dev/null | awk '{print $4}' | grep -qx "${ip}/24"; then
    ok "地址已存在：${iface}  ${ip}/24（不重复添加）"
    return 0
  fi
  if confirm "是否现在把 ${ip}/24 加到网卡 ${iface}？" Y; then
    run_root ip addr add "${ip}/24" dev "$iface" || warn "添加地址失败（可能已存在）"
    if $DRY_RUN; then
      info "dry-run：不会真正添加地址"
    elif ip -4 -o addr show dev "$iface" 2>/dev/null | awk '{print $4}' | grep -qx "${ip}/24"; then
      ok "主机 IP 已生效：${iface}  ${ip}/24"
    else
      warn "地址尚未出现在 ${iface} 上"
    fi
    if command -v nmcli >/dev/null 2>&1; then
      local conn
      conn="$(nmcli -t -f NAME,DEVICE connection show --active 2>/dev/null | awk -F: -v d="$iface" '$2==d {print $1; exit}')"
      if [[ -n $conn ]] && confirm "是否用 nmcli 持久化到连接「${conn}」？" Y; then
        run_root nmcli connection modify "$conn" +ipv4.addresses "${ip}/24" || warn "nmcli 持久化失败"
      elif [[ -z $conn ]]; then
        info "没有 NetworkManager 活跃连接；重启后会丢失，可用 netplan 自行持久化"
      fi
    fi
  else
    warn "已跳过地址配置；写雷达配置前请确保主机 IP 可用"
    SKIPPED+=("主机地址配置（用户跳过）")
  fi
}

# 从 IPv4 推导 /24 网段前缀（a.b.c）
subnet_of() { local ip="$1"; printf '%s' "${ip%.*}"; }

model_name() { # <dev_type>：Livox-SDK2 的 lidar_type
  case "$1" in
    "") ;;
    1) printf 'MID-40' ;;
    2) printf 'Tele-15' ;;
    3) printf 'Horizon' ;;
    6) printf 'MID-70' ;;
    7) printf 'Avia' ;;
    9) printf 'MID-360' ;;
    10) printf 'Industrial HAP' ;;
    15) printf 'HAP' ;;
    16) printf 'PA' ;;
    35) printf 'MID-360S' ;;
    40) printf 'Avia2' ;;
    *) printf '未知型号(dev_type=%s)' "$1" ;;
  esac
}

normalize_lidar_model() { # <型号文本> → MID-360 / MID-360S；无法识别返回 1
  local text="${1^^}"
  text="${text//_/-}"
  case "$text" in
    *360S*) printf 'MID-360S' ;;
    *360*) printf 'MID-360' ;;
    *) return 1 ;;
  esac
}

lidar_driver_launch_name() { # <MID-360|MID-360S> → Livox 驱动 launch 文件名
  case "$1" in
    MID-360S) printf 'msg_MID360s_launch.py' ;;
    MID-360) printf 'msg_MID360_launch.py' ;;
    *) return 1 ;;
  esac
}

patch_driver_launch_file() { # <file> <目标 launch 文件名>；输出 CHG/SAME/ERR
  python3 - "$1" "$2" <<'PY_DRIVER_LAUNCH'
import pathlib
import re
import shutil
import sys

path = pathlib.Path(sys.argv[1])
target = sys.argv[2]
pattern = re.compile(r"msg_MID360s?_launch\.py")
try:
    text = path.read_text(encoding="utf-8")
except OSError as exc:
    print(f"ERR 读取失败：{exc}")
    sys.exit(3)
found = sorted(set(pattern.findall(text)))
if not found:
    print("ERR 未找到 msg_MID360*_launch.py 引用")
    sys.exit(4)
if found == [target]:
    print(f"SAME {target}")
    raise SystemExit(0)
backup = path.with_name(path.name + ".bak")
try:
    if not backup.exists():
        shutil.copy2(path, backup)
    path.write_text(pattern.sub(target, text), encoding="utf-8")
except OSError as exc:
    print(f"ERR 写入失败：{exc}")
    sys.exit(5)
print(f"CHG {', '.join(found)} → {target}")
PY_DRIVER_LAUNCH
}

select_driver_launch() {
  local model target f out rc
  local -a files=()

  # 命令行走 --lidar-model 时以它为准；否则用探测到的型号。
  if [[ -n $LIDAR_MODEL_ARG ]]; then
    LIDAR_MODEL="$LIDAR_MODEL_ARG"
  fi
  if [[ -z ${LIDAR_MODEL:-} ]]; then
    skip "雷达型号未知，保留现有驱动 launch（可加 --lidar-model 指定）"
    return 0
  fi
  if ! model="$(normalize_lidar_model "$LIDAR_MODEL")"; then
    warn "无法识别的雷达型号：${LIDAR_MODEL}，保留现有驱动 launch"
    SKIPPED+=("驱动 launch 选择（型号无法识别）")
    return 0
  fi
  LIDAR_MODEL="$model"
  target="$(lidar_driver_launch_name "$model")"
  for f in "${LIDAR_DRIVER_LAUNCH_FILES[@]}"; do
    [[ -f $f ]] && files+=("$f")
  done
  if (( ${#files[@]} == 0 )); then
    warn "没有找到驱动 launch 文件（$WS/src/bw_fastlio_nav2/launch）"
    SKIPPED+=("驱动 launch 选择（文件缺失）")
    return 0
  fi

  printf '\n'
  printf '  雷达型号    : %s\n' "$model"
  printf '  驱动 launch : %s\n' "$target"
  for f in "${files[@]}"; do printf '  改写文件    : %s\n' "$f"; done

  if $DRY_RUN; then
    act "[dry-run] 按 ${model} 改写 ${#files[@]} 个 launch 文件中的驱动 launch 名称"
    return 0
  fi
  if ! confirm "是否按雷达型号改写驱动 launch 名称？" Y; then
    warn "已跳过驱动 launch 选择（360 与 360S 的驱动 launch 不同）"
    SKIPPED+=("驱动 launch 选择（用户跳过）")
    return 0
  fi
  for f in "${files[@]}"; do
    rc=0
    out="$(patch_driver_launch_file "$f" "$target")" || rc=$?
    if (( rc == 0 )); then
      case "$out" in
        "SAME "*) skip "已经是 ${target}：$f" ;;
        "CHG "*) ok "${out#CHG }：$f" ;;
      esac
    else
      if [[ $out == "ERR "* ]]; then
        warn "${out#ERR }：$f"
      else
        warn "改写失败（退出码 ${rc}）：$f"
      fi
      ERRORS+=("驱动 launch 改写失败：$f")
    fi
  done
}

lidar_label() { # <ip|dev|sn>：展示用；永远返回 0（会用在 $(...) 里）
  local raw="$1" ip rest dev sn m
  ip="${raw%%|*}"
  dev=""; sn=""
  if [[ $raw == *"|"* ]]; then
    rest="${raw#*|}"; dev="${rest%%|*}"
    if [[ $rest == *"|"* ]]; then sn="${rest#*|}"; sn="${sn%%|*}"; fi
  fi
  m="$(model_name "$dev")"
  if [[ -n $m ]]; then
    printf '%s（%s）' "$ip" "$m"
  else
    printf '%s（未识别）' "$ip"
  fi
  if [[ -n $sn ]]; then printf '，SN %s' "$sn"; fi
  return 0
}

lidar_network_tool() {
  # Linux packet sockets bypass IPv4 route/rp_filter restrictions during discovery.
  python3 - "$@" <<'PY_NET'
import binascii
import concurrent.futures
import ipaddress
import itertools
import json
import pathlib
import secrets
import socket
import struct
import subprocess
import sys
import time
import zlib


def choose_subnet(addresses, routes):
    occupied = []
    for interface in addresses:
        for addr in interface.get("addr_info", []):
            if addr.get("family") == "inet":
                occupied.append(ipaddress.ip_network(
                    "%s/%s" % (addr["local"], addr["prefixlen"]), strict=False))
    for route in routes:
        dst = route.get("dst", "default")
        if dst not in ("default", "0.0.0.0/0"):
            occupied.append(ipaddress.ip_network(dst, strict=False))
    candidates = itertools.chain(
        ("192.168.%d.0/24" % n for n in itertools.chain(range(10, 255), range(2, 10), [0, 255])),
        ("172.%d.%d.0/24" % (a, b) for a in range(16, 32) for b in range(256)),
        ("10.%d.%d.0/24" % (a, b) for a in range(256) for b in range(256)))
    for candidate in candidates:
        net = ipaddress.ip_network(candidate)
        if not any(net.overlaps(other) for other in occupied):
            return str(net.network_address).rsplit(".", 1)[0]
    raise ValueError("No unused private /24 subnet; check VPN and interface routes")


def search_packet(seq):
    header = struct.pack("<BBHIHBB6s", 0xAA, 0, 24, seq, 0, 0, 0, b"\0" * 6)
    return header + struct.pack("<HI", binascii.crc_hqx(header, 0xFFFF), 0)


def decode_reply(data, seq):
    if len(data) < 48 or data[:2] != b"\xaa\x00":
        return None
    size, received_seq, cmd, kind, sender = struct.unpack_from("<HIHBB", data, 2)
    if size != len(data) or (received_seq, cmd, kind, sender) != (seq, 0, 1, 1):
        return None
    crc16, crc32 = struct.unpack_from("<HI", data, 18)
    if crc16 != binascii.crc_hqx(data[:18], 0xFFFF) or crc32 != zlib.crc32(data[24:]):
        return None
    if data[24] != 0:
        return None
    address = ipaddress.ip_address(data[42:46])
    if address.is_unspecified or address.is_multicast or address.is_loopback:
        return None
    sn = data[26:42].split(b"\0", 1)[0]
    if not sn or not all(33 <= c <= 126 and c != 124 for c in sn):
        return None
    return str(address), data[25], sn.decode("ascii")


def checksum(data):
    total = sum(struct.unpack("!%dH" % (len(data) // 2), data))
    while total >> 16:
        total = (total & 0xFFFF) + (total >> 16)
    return (~total) & 0xFFFF


def udp_payload(frame, port):
    if len(frame) < 42 or frame[12:14] != b"\x08\x00":
        return None
    ip = frame[14:]
    ihl = (ip[0] & 15) * 4
    if ip[0] >> 4 != 4 or ihl < 20 or ip[9] != 17 or len(ip) < ihl + 8:
        return None
    if struct.unpack_from("!H", ip, 6)[0] & 0x3FFF:
        return None  # Discovery packets must not be fragmented.
    total = struct.unpack_from("!H", ip, 2)[0]
    src, dst, size = struct.unpack_from("!HHH", ip, ihl)
    if dst != port or size < 8 or ihl + size > total or total > len(ip):
        return None
    return ip[ihl + 8:ihl + size]


def discover_interface(iface, timeout, port):
    mac = bytes.fromhex(pathlib.Path("/sys/class/net", iface, "address").read_text().strip().replace(":", ""))
    addresses = json.loads(subprocess.check_output(["ip", "-j", "-4", "addr", "show", "dev", iface]))
    source = next((a["local"] for i in addresses for a in i.get("addr_info", [])
                   if a.get("family") == "inet"), "0.0.0.0")
    seq = secrets.randbelow(65536)
    packet = search_packet(seq)
    udp = struct.pack("!HHHH", port, port, len(packet) + 8, 0) + packet
    header = struct.pack("!BBHHHBBH4s4s", 0x45, 0, 20 + len(udp), seq, 0,
                         64, 17, 0, socket.inet_aton(source), b"\xff" * 4)
    header = header[:10] + struct.pack("!H", checksum(header)) + header[12:]
    frame = b"\xff" * 6 + mac + b"\x08\x00" + header + udp
    found = {}
    # A raw socket can receive replies from any subnet, including interfaces without IPv4.
    with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0800)) as sock:
        sock.bind((iface, 0))
        sock.settimeout(0.15)
        deadline, next_send = time.monotonic() + timeout, 0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_send:
                sock.send(frame)
                next_send = time.monotonic() + 0.5
            try:
                received, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            if addr[2] == socket.PACKET_OUTGOING:
                continue
            payload = udp_payload(received, port)
            identity = decode_reply(payload, seq) if payload else None
            if identity:
                found[identity] = (*identity, iface)
    return list(found.values())


def discover_all(ifaces, timeout, port):
    found = []
    if not ifaces:
        return found
    with concurrent.futures.ThreadPoolExecutor(max_workers=min(len(ifaces), 32)) as pool:
        futures = {pool.submit(discover_interface, i, timeout, port): i for i in ifaces}
        for future in concurrent.futures.as_completed(futures):
            try:
                found.extend(future.result())
            except OSError as exc:
                print("Discovery %s: %s" % (futures[future], exc), file=sys.stderr)
    return sorted(set(found))


if __name__ == "__main__":
    try:
        if sys.argv[1] == "plan":
            print(choose_subnet(json.loads(sys.argv[2]), json.loads(sys.argv[3])))
        elif sys.argv[1] == "discover":
            for row in discover_all(sys.argv[4:], float(sys.argv[2]), int(sys.argv[3])):
                print("|".join(map(str, row)))
        else:
            raise ValueError("Unknown network operation")
    except (OSError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
PY_NET
}

scan_all_radars() {
  local -a ifaces=()
  if (( $# )); then ifaces=("$@"); else mapfile -t ifaces < <(wired_ifaces); fi
  (( ${#ifaces[@]} )) || return 0
  if $DRY_RUN; then
    info "[dry-run] 并行发现有线接口：${ifaces[*]}（任意 IPv4 网段）" >&2
    return 0
  fi
  # Functions cannot be passed to sudo; execute only the embedded read-only discovery helper.
  if [[ $EUID -eq 0 ]]; then
    lidar_network_tool discover "${LIDAR_UDP_TIMEOUT:-3}" "$LIDAR_UDP_PORT" "${ifaces[@]}"
  else
    sudo bash -c "$(declare -f lidar_network_tool); lidar_network_tool \"\$@\"" bash \
      discover "${LIDAR_UDP_TIMEOUT:-3}" "$LIDAR_UDP_PORT" "${ifaces[@]}"
  fi
}

is_wifi_iface() { [[ -e "/sys/class/net/$1/wireless" ]]; }

wifi_on_subnet() { # <a.b.c>：打印与 a.b.c.0/24 网段重叠的无线地址 "iface ip/prefix"
  local subnet="$1" ifc cidr addrs=""
  [[ -n $subnet ]] || return 1
  while read -r ifc cidr; do
    [[ -n $ifc && -n $cidr ]] || continue
    is_wifi_iface "$ifc" || continue
    addrs+="$ifc $cidr"$'\n'
  done < <(ip -4 -o addr show 2>/dev/null | awk '$3 == "inet" {print $2, $4}')
  [[ -n $addrs ]] || return 1
  printf '%s' "$addrs" | python3 -c '
import ipaddress, sys
subnet = ipaddress.ip_network(sys.argv[1] + ".0/24")
found = False
for line in sys.stdin:
    parts = line.split()
    if len(parts) != 2:
        continue
    iface, cidr = parts
    try:
        network = ipaddress.ip_interface(cidr).network
    except ValueError:
        continue
    if network.overlaps(subnet):
        print(iface, cidr)
        found = True
sys.exit(0 if found else 1)
' "$subnet"
}

select_migration_subnet() {
  local addresses routes
  addresses="$(ip -j -4 addr show)" || return 1
  routes="$(ip -j -4 route show table all)" || return 1
  lidar_network_tool plan "$addresses" "$routes"
}

build_lidar_ip_setter() { # <temporary directory>; use the installed SDK, not an external demo
  local dir="$1"
  cat > "$dir/set_ip.cpp" <<'CPP'
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

static uint32_t target;
static const char* serial;
static LivoxLidarIpInfo config{};
static std::atomic<bool> sent{false}, applied{false};
static std::atomic<int> result{-1};

static void rebooted(livox_status status, uint32_t, LivoxLidarRebootResponse* reply, void*) {
  result = (status == kLivoxLidarStatusSuccess && reply && reply->ret_code == 0) ? 0 : 3;
}
static void configured(livox_status status, uint32_t handle,
                       LivoxLidarAsyncControlResponse* reply, void*) {
  if (status != kLivoxLidarStatusSuccess || !reply || reply->ret_code || reply->error_key) {
    std::fprintf(stderr, "Set IP failed or acknowledgement lost (status=%d)\n", status);
    result = 2;
    return;
  }
  applied = true;
  std::puts("SET_IP_ACK_OK");
  if (LivoxLidarRequestReboot(handle, rebooted, nullptr) != kLivoxLidarStatusSuccess)
    result = 3;
}
static void online(uint32_t handle, const LivoxLidarInfo* info, void*) {
  if (!info || handle != target || std::strncmp(info->sn, serial, sizeof(info->sn)) != 0)
    return;
  if (sent.exchange(true)) return;
  if (SetLivoxLidarIp(handle, &config, configured, nullptr) != kLivoxLidarStatusSuccess)
    result = 2;
}
int main(int argc, char** argv) {
  if (argc != 6) return 64;  // config, old IP, serial, new IP, gateway
  in_addr address{};
  if (inet_pton(AF_INET, argv[2], &address) != 1) return 64;
  target = address.s_addr;
  serial = argv[3];
  if (inet_pton(AF_INET, argv[4], &address) != 1 ||
      inet_pton(AF_INET, argv[5], &address) != 1) return 64;
  std::snprintf(config.ip_addr, sizeof(config.ip_addr), "%s", argv[4]);
  std::snprintf(config.net_mask, sizeof(config.net_mask), "255.255.255.0");
  std::snprintf(config.gw_addr, sizeof(config.gw_addr), "%s", argv[5]);
  if (!LivoxLidarSdkInit(argv[1])) return 1;
  SetLivoxLidarInfoChangeCallback(online, nullptr);
  for (int i = 0; i < 300 && result.load() < 0; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  LivoxLidarSdkUninit();
  // Reboot ACK may disappear as the device restarts; the caller MUST rediscover its SN.
  if (applied.load()) return 0;
  return result.load() < 0 ? 4 : result.load();
}
CPP
  g++ -std=c++11 "$dir/set_ip.cpp" -I"$(dirname "$LIVOX_HEADER")" \
    "$LIVOX_STATIC_LIB" -pthread -o "$dir/set_ip"
}

verify_migrated_lidar() { # <new IP> <SN> <iface>
  local attempt row ip dev sn iface
  for attempt in {1..10}; do
    while IFS='|' read -r ip dev sn iface; do
      if [[ $ip == "$1" && $sn == "$2" && $iface == "$3" ]]; then return 0; fi
    done < <(scan_all_radars "$3")
    sleep 1
  done
  return 1
}

save_migration_state() { # iface, old_ip, serial, dev, new_subnet, new_host, new_lidar
  python3 - "$MIGRATION_STATE" "$@" <<'PY_STATE'
import json, os, sys, tempfile
path = sys.argv[1]
fd, temporary = tempfile.mkstemp(dir=os.path.dirname(path), prefix=".lidar-migration-")
try:
    with os.fdopen(fd, "w") as output:
        json.dump(sys.argv[2:], output)
        output.flush()
        os.fsync(output.fileno())
    os.chmod(temporary, 0o644)
    os.replace(temporary, path)
finally:
    if os.path.exists(temporary):
        os.unlink(temporary)
PY_STATE
}

read_migration_state() {
  python3 - "$MIGRATION_STATE" <<'PY_STATE'
import ipaddress, json, re, sys
with open(sys.argv[1]) as source:
    values = json.load(source)
if not isinstance(values, list) or len(values) != 7 or not all(isinstance(v, str) for v in values):
    raise ValueError("Invalid migration journal")
iface, old, serial, dev, subnet, host, lidar = values
if not re.fullmatch(r"[A-Za-z0-9_.:-]{1,15}", iface) or dev not in ("9", "35"):
    raise ValueError("Invalid migration interface/device")
if not re.fullmatch(r"[!-~]{1,16}", serial) or "|" in serial:
    raise ValueError("Invalid serial")
old_address = ipaddress.IPv4Address(old)
old_network = ipaddress.IPv4Network(old + "/24", strict=False)
network = ipaddress.IPv4Network(subnet + ".0/24")
if old_address.is_multicast or old_address.is_loopback or old_address.is_unspecified:
    raise ValueError("Invalid old IP")
if not network.is_private or network.overlaps(old_network):
    raise ValueError("Invalid migration subnet")
for value in (host, lidar):
    address = ipaddress.IPv4Address(value)
    if address not in network or address in (network.network_address, network.broadcast_address):
        raise ValueError("Invalid new IP")
if host == lidar:
    raise ValueError("Host and lidar IP collide")
print("|".join(values))
PY_STATE
}

persist_migrated_network() { # iface, old IP, new host, new lidar, serial
  local iface="$1" old_ip="$2" new_host="$3" new_lidar="$4" serial="$5" conn cidr
  local old_sub="${old_ip%.*}"
  conn="$(nmcli -g GENERAL.CON-UUID device show "$iface")" || return 1
  [[ -n $conn && $conn != -- ]] || { err "$iface 没有活跃的 NetworkManager 连接"; return 1; }
  if ! ip -4 -o addr show dev "$iface" | awk '{print $4}' | grep -qx "$new_host/24"; then
    run_root arping -D -q -c 2 -w 2 -I "$iface" "$new_host" || return 1
    run_root ip addr add "$new_host/24" dev "$iface" || return 1
  fi
  verify_migrated_lidar "$new_lidar" "$serial" "$iface" || {
    err "未在新 IP 验证到同一序列号的雷达"; return 1;
  }
  run_root nmcli connection modify uuid "$conn" ipv4.method manual \
    ipv4.addresses "$new_host/24" ipv4.gateway "" ipv4.routes "" \
    ipv4.dns "" ipv4.ignore-auto-dns yes ipv4.never-default yes || return 1
  run_root nmcli device reapply "$iface" || return 1
  if ip -4 route show exact "$old_ip/32" | grep -q "dev $iface"; then
    run_root ip route del "$old_ip/32" dev "$iface" || return 1
  fi
  while read -r cidr; do
    if [[ ${cidr%/*} == "$old_sub".* ]]; then run_root ip addr del "$cidr" dev "$iface" || return 1; fi
  done < <(ip -4 -o addr show dev "$iface" | awk '$3 == "inet" {print $4}')
  ip -4 -o addr show dev "$iface" | awk '{print $4}' | grep -qx "$new_host/24" || return 1
  verify_migrated_lidar "$new_lidar" "$serial" "$iface" || return 1
  rm -f -- "$MIGRATION_STATE"
}

resume_lidar_migration() {
  local state iface old_ip serial dev new_sub new_host new_lidar ip found_dev sn found_iface current=""
  state="$(read_migration_state)" || { err "迁移记录无效：$MIGRATION_STATE"; return 1; }
  IFS='|' read -r iface old_ip serial dev new_sub new_host new_lidar <<<"$state"
  act "恢复未完成的迁移：SN $serial，$old_ip → $new_lidar（$iface）"
  if $DRY_RUN; then
    info "[dry-run] 验证雷达当前地址，完成有线配置持久化并清理旧地址"
  else
    ensure_sudo
    is_wired_iface "$iface" || { err "迁移记录中的有线接口不存在：$iface"; return 1; }
    while IFS='|' read -r ip found_dev sn found_iface; do
      if [[ $sn == "$serial" && $found_dev == "$dev" && $found_iface == "$iface" ]]; then
        [[ -z $current ]] || { err "迁移目标身份不唯一"; return 1; }
        current="$ip"
      fi
    done < <(scan_all_radars "$iface")
    if [[ $current == "$new_lidar" ]]; then
      persist_migrated_network "$iface" "$old_ip" "$new_host" "$new_lidar" "$serial" || return 1
    elif [[ $current == "$old_ip" ]]; then
      migration_transaction "$iface" "$old_ip" "$serial" "$dev" "$new_sub" "$new_host" "$new_lidar" || return 1
    else
      err "未发现迁移记录中的雷达，请检查连接后重试（记录已保留）"; return 1
    fi
  fi
  HOST_IFACE="$iface"; HOST_IP="$new_host"; HOST_CIDR="$new_host/24"
  LIDAR_IP="$new_lidar"; LIDAR_SN="$serial"; LIDAR_IFACE="$iface"; SUBNET="$new_sub"
  LIDAR_MODEL="$(model_name "$dev")"
}

migration_transaction() ( # isolate temporary-file cleanup from the installer EXIT trap
  local iface="$1" old_ip="$2" serial="$3" dev="$4" new_sub="$5" new_host="$6" new_lidar="$7"
  local old_sub="${old_ip%.*}"
  local dir conn old_host="" cidr candidate rc old_added=false route_added=false new_added=false write_started=false complete=false
  command -v nmcli >/dev/null && command -v arping >/dev/null || {
    err "自动迁移需要 NetworkManager/nmcli 和 arping"; return 1;
  }
  conn="$(nmcli -g GENERAL.CON-UUID device show "$iface")" || return 1
  [[ -n $conn && $conn != -- ]] || { err "$iface 没有活跃的 NetworkManager 连接，停止迁移"; return 1; }
  [[ -f $LIVOX_HEADER && -f $LIVOX_STATIC_LIB ]] || { err "缺少 Livox-SDK2，不能写入雷达 IP"; return 1; }
  if [[ -n $(ss -H -lun '( sport = :56000 or sport = :56101 or sport = :56201 or sport = :56301 or sport = :56401 or sport = :56501 )') ]]; then
    err "Livox UDP 端口已占用，请先停止雷达驱动或其他 SDK 程序"; return 1
  fi
  dir="$(mktemp -d)" || return 1
  transaction_cleanup() {
    # Before sending a write, remove only temporary state created by this transaction.
    if ! $write_started; then
      if $route_added; then run_root ip route del "$old_ip/32" dev "$iface" >/dev/null 2>&1 || true; fi
      if $old_added; then run_root ip addr del "$old_host/32" dev "$iface" >/dev/null 2>&1 || true; fi
      if $new_added; then run_root ip addr del "$new_host/24" dev "$iface" >/dev/null 2>&1 || true; fi
    elif ! $complete; then
      err "迁移未完成：雷达可能位于 $old_ip 或 $new_lidar（SN $serial）。"
      err "保留 $iface 上的临时地址以便恢复；检查雷达/重启后重新运行脚本。未写入 ROS 配置。"
    fi
    rm -rf -- "$dir"
  }
  trap transaction_cleanup EXIT
  build_lidar_ip_setter "$dir" || return 1
  while read -r cidr; do
    if [[ ${cidr%/*} == "$old_sub".* && ${cidr%/*} != "$old_ip" ]]; then old_host="${cidr%/*}"; break; fi
  done < <(ip -4 -o addr show dev "$iface" | awk '$3 == "inet" {print $4}')
  if [[ -z $old_host ]]; then
    for candidate in 5 6 250 251 252 253 254; do
      candidate="$old_sub.$candidate"
      [[ $candidate == "$old_ip" ]] && continue
      if local_ips | grep -qx "$candidate"; then continue; fi
      rc=0
      run_root arping -D -q -c 2 -w 2 -I "$iface" "$candidate" || rc=$?
      if (( rc == 0 )); then old_host="$candidate"; break; fi
      (( rc == 1 )) || { err "ARP 地址检查失败"; return 1; }
    done
    [[ -n $old_host ]] || { err "旧网段没有可用的临时主机地址"; return 1; }
    run_root ip addr add "$old_host/32" dev "$iface" || return 1
    old_added=true
  fi
  # A host route pins only this lidar to Ethernet without stealing Wi-Fi's /24 route.
  local existing_route saved_state=""
  existing_route="$(ip -4 route show exact "$old_ip/32")" || return 1
  if [[ -n $existing_route ]]; then
    [[ ! -f $MIGRATION_STATE ]] || saved_state="$(read_migration_state)" || return 1
    if [[ $saved_state != "$iface|$old_ip|$serial|$dev|$new_sub|$new_host|$new_lidar" \
          || " $existing_route " != *" dev $iface "* || " $existing_route " != *" src $old_host "* ]]; then
      err "已存在不属于本次迁移的 $old_ip/32 路由，停止迁移"; return 1
    fi
  else
    run_root ip route add "$old_ip/32" dev "$iface" src "$old_host" || return 1
    route_added=true
  fi
  for candidate in "$new_host" "$new_lidar"; do
    if [[ $candidate == "$new_host" ]] && ip -4 -o addr show dev "$iface" | awk '{print $4}' | grep -qx "$new_host/24"; then continue; fi
    run_root arping -D -q -c 2 -w 2 -I "$iface" "$candidate" || {
      err "目标地址 $candidate 已占用或 ARP 检查失败"; return 1;
    }
  done
  if ! ip -4 -o addr show dev "$iface" | awk '{print $4}' | grep -qx "$new_host/24"; then
    run_root ip addr add "$new_host/24" dev "$iface" || return 1
    new_added=true
  fi
  python3 - "$dir/config.json" "$dev" "$old_host" "$old_ip" <<'PY_CFG'
import json, sys
path, dev, host, lidar = sys.argv[1:]
ports = {"cmd_data_port": 56100, "push_msg_port": 56200, "point_data_port": 56300,
         "imu_data_port": 56400, "log_data_port": 56500}
host_ports = {k: v + 1 for k, v in ports.items()}
block = "Mid360s" if dev == "35" else "MID360"
# SDK host_net_info.lidar_ip restricts initialization to this device as well as the callback.
host_info = [dict(host_ip=host, lidar_ip=[lidar], **host_ports)]
with open(path, "w") as output:
    json.dump({block: {"lidar_net_info": ports, "host_net_info": host_info},
               "lidar_configs": [{"ip": lidar, "pcl_data_type": 1, "pattern_mode": 0}]}, output)
PY_CFG
  [[ $? == 0 ]] || return 1
  save_migration_state "$iface" "$old_ip" "$serial" "$dev" "$new_sub" "$new_host" "$new_lidar" || return 1
  write_started=true
  "$dir/set_ip" "$dir/config.json" "$old_ip" "$serial" "$new_lidar" "$new_sub.1" || return 1
  persist_migrated_network "$iface" "$old_ip" "$new_host" "$new_lidar" "$serial" || return 1
  complete=true
)

migrate_lidar_network() {
  local new_sub new_host new_lidar row ip dev sn iface
  local -a matches=()
  new_sub="$(select_migration_subnet)" || { err "找不到不冲突的新网段"; return 1; }
  new_lidar="$new_sub.${LIDAR_IP##*.}"
  case "${LIDAR_IP##*.}" in 0|255) new_lidar="$new_sub.156" ;; esac
  new_host="$new_sub.5"
  if [[ -n ${HOST_IP_ARG:-} ]]; then
    if valid_ipv4 "$HOST_IP_ARG" && [[ ${HOST_IP_ARG%.*} == "$new_sub" ]]; then
      new_host="$HOST_IP_ARG"
    else
      warn "--host-ip ${HOST_IP_ARG} 不在迁移目标网段 ${new_sub}.0/24，改用 ${new_host}"
    fi
  fi
  if [[ $new_host == "$new_lidar" ]]; then
    new_host="$new_sub.6"
    [[ $new_host == "$new_lidar" ]] && new_host="$new_sub.7"
  fi
  act "雷达网段 ${SUBNET}.0/24 与 Wi-Fi 冲突：雷达 $LIDAR_IP → $new_lidar，有线主机 → $new_host/24"
  if $DRY_RUN; then
    info "[dry-run] 校验雷达身份 → SDK 写入/重启 → 验证新地址 → nmcli 持久化有线连接"
  else
    ensure_sudo
    while IFS='|' read -r ip dev sn iface; do
      [[ $ip == "$LIDAR_IP" ]] && matches+=("$ip|$dev|$sn|$iface")
    done < <(scan_all_radars)
    (( ${#matches[@]} == 1 )) || { err "无法唯一确认 $LIDAR_IP 对应的雷达/有线接口，停止自动迁移"; return 1; }
    IFS='|' read -r ip dev sn iface <<<"${matches[0]}"
    [[ $dev == 35 || $dev == 9 ]] || { err "自动迁移仅支持 MID360S/MID360（检测到 $dev）"; return 1; }
    HOST_IFACE="$iface"; LIDAR_IFACE="$iface"; LIDAR_SN="$sn"; LIDAR_MODEL="$(model_name "$dev")"
    if ! migration_transaction "$iface" "$LIDAR_IP" "$sn" "$dev" "$new_sub" "$new_host" "$new_lidar"; then
      ERRORS+=("雷达/有线 IP 自动迁移失败")
      return 1
    fi
  fi
  HOST_IP="$new_host"; HOST_CIDR="$new_host/24"; LIDAR_IP="$new_lidar"; SUBNET="$new_sub"
  info "迁移目标：雷达 $LIDAR_IP，主机 $HOST_IP（${HOST_IFACE:-待发现的有线接口}）"
}

livox_ports_free() { # SDK 发现/控制要用的 UDP 端口必须空闲
  [[ -z "$(ss -H -lun '( sport = :56000 or sport = :56101 or sport = :56201 or sport = :56301 or sport = :56401 or sport = :56501 )')" ]]
}

private_subnet() { # <a.b.c>：跨网段迁移只允许私有 /24（与迁移记录校验一致）
  python3 -c 'import ipaddress, sys; sys.exit(0 if ipaddress.ip_network(sys.argv[1] + ".0/24").is_private else 1)' "$1"
}

refresh_lidar_identity() { # 按当前 LIDAR_IP 重新扫描，补全 SN / 型号 / 有线网卡
  local ip dev sn iface
  local -a matches=() scan_ifaces=()
  [[ -n ${LIDAR_IFACE:-} ]] && scan_ifaces+=("$LIDAR_IFACE")
  ensure_sudo
  while IFS='|' read -r ip dev sn iface; do
    [[ -n $ip ]] || continue
    [[ $ip == "$LIDAR_IP" ]] && matches+=("$ip|$dev|$sn|$iface")
  done < <(scan_all_radars "${scan_ifaces[@]}")
  if (( ${#matches[@]} != 1 )); then
    warn "无法唯一确认 ${LIDAR_IP} 对应的雷达（匹配 ${#matches[@]} 个）"
    return 1
  fi
  IFS='|' read -r ip dev sn iface <<<"${matches[0]}"
  LIDAR_IFACE="$iface"; LIDAR_SN="$sn"; LIDAR_DEV_TYPE="$dev"
  LIDAR_MODEL="$(model_name "$dev")"
  if [[ $dev != 9 && $dev != 35 ]]; then
    warn "自动改 IP 仅支持 MID360/MID360S（检测到 dev_type=${dev}）"
    return 1
  fi
  ok "已核验雷达：$(lidar_label "$ip|$dev|$sn")，网卡 ${LIDAR_IFACE}"
}

default_migration_host() { # <新网段前缀> <新雷达IP> → 新网段主机地址
  local new_sub="$1" new_lidar="$2" new_host="$1.5"
  if [[ -n ${HOST_IP_ARG:-} ]] && valid_ipv4 "$HOST_IP_ARG" \
     && [[ ${HOST_IP_ARG%.*} == "$new_sub" && $HOST_IP_ARG != "$new_lidar" ]]; then
    new_host="$HOST_IP_ARG"
  fi
  [[ $new_host == "$new_lidar" ]] && new_host="$new_sub.6"
  [[ $new_host == "$new_lidar" ]] && new_host="$new_sub.7"
  printf '%s' "$new_host"
}

change_lidar_ip_same_subnet() ( # <iface> <old_ip> <serial> <dev> <new_ip>
  # 同网段改写：只动雷达 IP，主机地址/路由保持不变；SDK 重启后按 SN 验证。
  local iface="$1" old_ip="$2" serial="$3" dev="$4" new_ip="$5"
  local dir host="" cidr candidate rc existing_route
  local added_host=false route_added=false write_started=false complete=false
  [[ -f $LIVOX_HEADER && -f $LIVOX_STATIC_LIB ]] || {
    err "缺少 Livox-SDK2，不能写入雷达 IP"; return 1;
  }
  livox_ports_free || {
    err "Livox UDP 端口已占用，请先停止雷达驱动或其他 SDK 程序"; return 1;
  }
  dir="$(mktemp -d)" || return 1
  same_subnet_cleanup() {
    # 写雷达前失败只清理本轮临时地址；写后失败则保留，便于确认雷达落在哪个 IP。
    if ! $write_started; then
      if $route_added; then run_root ip route del "$old_ip/32" dev "$iface" >/dev/null 2>&1 || true; fi
      if $added_host; then run_root ip addr del "$host/32" dev "$iface" >/dev/null 2>&1 || true; fi
    elif ! $complete; then
      err "写入未完成：雷达可能位于 ${old_ip} 或 ${new_ip}（SN ${serial}）。"
      err "保留 ${iface} 上的临时地址以便恢复；请确认雷达状态后重试。"
    fi
    rm -rf -- "$dir"
  }
  trap same_subnet_cleanup EXIT
  build_lidar_ip_setter "$dir" || return 1
  while read -r cidr; do
    cidr="${cidr%/*}"
    if [[ $cidr == "$SUBNET".* && $cidr != "$old_ip" && $cidr != "$new_ip" ]]; then
      host="$cidr"; break
    fi
  done < <(ip -4 -o addr show dev "$iface" | awk '$3 == "inet" {print $4}')
  if [[ -z $host ]]; then
    for candidate in 5 6 250 251 252 253 254; do
      candidate="$SUBNET.$candidate"
      [[ $candidate == "$old_ip" || $candidate == "$new_ip" ]] && continue
      if local_ips | grep -qx "$candidate"; then continue; fi
      rc=0
      run_root arping -D -q -c 2 -w 2 -I "$iface" "$candidate" || rc=$?
      if (( rc == 0 )); then host="$candidate"; break; fi
      (( rc == 1 )) || { err "ARP 地址检查失败"; return 1; }
    done
    [[ -n $host ]] || { err "同网段没有可用的临时主机地址"; return 1; }
    run_root ip addr add "$host/32" dev "$iface" || return 1
    added_host=true
  fi
  # 主机路由把去往旧雷达 IP 的流量固定在有线口，避免走 Wi-Fi 的同网段路由。
  existing_route="$(ip -4 route show exact "$old_ip/32")" || return 1
  if [[ -z $existing_route ]]; then
    run_root ip route add "$old_ip/32" dev "$iface" src "$host" || return 1
    route_added=true
  fi
  run_root arping -D -q -c 2 -w 2 -I "$iface" "$new_ip" || {
    err "目标地址 ${new_ip} 已占用或 ARP 检查失败"; return 1;
  }
  python3 - "$dir/config.json" "$dev" "$host" "$old_ip" <<'PY_CFG'
import json, sys
path, dev, host, lidar = sys.argv[1:]
ports = {"cmd_data_port": 56100, "push_msg_port": 56200, "point_data_port": 56300,
         "imu_data_port": 56400, "log_data_port": 56500}
host_ports = {k: v + 1 for k, v in ports.items()}
block = "Mid360s" if dev == "35" else "MID360"
host_info = [dict(host_ip=host, lidar_ip=[lidar], **host_ports)]
with open(path, "w") as output:
    json.dump({block: {"lidar_net_info": ports, "host_net_info": host_info},
               "lidar_configs": [{"ip": lidar, "pcl_data_type": 1, "pattern_mode": 0}]}, output)
PY_CFG
  write_started=true
  "$dir/set_ip" "$dir/config.json" "$old_ip" "$serial" "$new_ip" "$SUBNET.1" || return 1
  verify_migrated_lidar "$new_ip" "$serial" "$iface" || return 1
  complete=true
  if $route_added; then run_root ip route del "$old_ip/32" dev "$iface" >/dev/null 2>&1 || true; fi
  if $added_host; then run_root ip addr del "$host/32" dev "$iface" >/dev/null 2>&1 || true; fi
  return 0
)

maybe_change_lidar_ip() { # 侦测到雷达 IP 后询问是否经 SDK 改写
  local target target_sub new_host tries=0
  [[ -n ${LIDAR_IP:-} ]] || return 0
  if ! confirm "是否修改雷达 IP（当前 ${LIDAR_IP}）？" N; then
    info "保持雷达 IP：${LIDAR_IP}"
    return 0
  fi
  while (( tries < 3 )); do
    target="$(ask "请输入新的雷达 IP（当前 ${LIDAR_IP}）: " "")"
    if ! valid_ipv4 "$target"; then
      warn "不是合法的 IPv4：${target:-（空）}"
    elif [[ $target =~ (^|\.)0[0-9] ]]; then
      warn "IP 不能包含前导零：$target"
    elif [[ $target == "$LIDAR_IP" ]]; then
      warn "新 IP 与当前 IP 相同：$target"
    elif [[ ${target##*.} == 0 || ${target##*.} == 255 ]]; then
      warn "不能使用网络地址或广播地址：$target"
    elif local_ips | grep -qx "$target"; then
      warn "新 IP 是本机地址：$target"
    else
      target_sub="$(subnet_of "$target")"
      if [[ $target_sub != "$SUBNET" ]] && ! private_subnet "$target_sub"; then
        warn "跨网段目标必须是私有网段：$target"
      else
        break
      fi
    fi
    target=""; tries=$((tries + 1))
  done
  if [[ -z $target ]]; then
    warn "未获得有效的新雷达 IP，保持 ${LIDAR_IP}"
    return 0
  fi
  target_sub="$(subnet_of "$target")"
  if $DRY_RUN; then
    if [[ $target_sub == "$SUBNET" ]]; then
      info "[dry-run] 通过 SDK 将雷达 ${LIDAR_IP} → ${target}（同网段），重启后按 SN 验证"
    else
      new_host="$(default_migration_host "$target_sub" "$target")"
      info "[dry-run] 通过 SDK 将雷达 ${LIDAR_IP} → ${target}；主机 ${new_host}/24；nmcli 持久化"
      HOST_IP="$new_host"; HOST_CIDR="$new_host/24"
    fi
    LIDAR_IP="$target"; SUBNET="$target_sub"; LIDAR_IP_CHANGED=true
    return 0
  fi
  ensure_sudo
  if ! refresh_lidar_identity; then
    err "无法确认雷达身份，取消修改并停止部署；雷达 IP 保持 ${LIDAR_IP}"
    return 1
  fi
  if [[ $target_sub == "$SUBNET" ]]; then
    act "同网段修改雷达 IP：${LIDAR_IP} → ${target}"
    if ! change_lidar_ip_same_subnet "$LIDAR_IFACE" "$LIDAR_IP" "$LIDAR_SN" "$LIDAR_DEV_TYPE" "$target"; then
      err "雷达 IP 修改失败，停止部署；仍为 ${LIDAR_IP}"
      return 1
    fi
  else
    new_host="$(default_migration_host "$target_sub" "$target")"
    if [[ -n $(wifi_on_subnet "$target_sub" || true) ]]; then
      warn "新网段 ${target_sub}.0/24 与 Wi-Fi 重叠；按指定 IP 继续，但雷达数据可能走错接口"
    fi
    act "跨网段修改雷达 IP：${LIDAR_IP} → ${target}，主机 ${new_host}/24"
    if ! migration_transaction "$LIDAR_IFACE" "$LIDAR_IP" "$LIDAR_SN" "$LIDAR_DEV_TYPE" \
         "$target_sub" "$new_host" "$target"; then
      err "雷达 IP 修改失败，停止部署；雷达可能位于 ${LIDAR_IP} 或 ${target}"
      return 1
    fi
    HOST_IP="$new_host"; HOST_CIDR="$new_host/24"
  fi
  LIDAR_IP="$target"; SUBNET="$target_sub"; LIDAR_IP_CHANGED=true
  ok "雷达 IP 已修改：${LIDAR_IP}（SN ${LIDAR_SN}）"
}

ask_radar_ip() { # 手动输入雷达 IP；成功设置 LIDAR_IP / SUBNET
  if $ASSUME_YES || [[ ! -t 0 ]]; then return 1; fi
  local ans tries=0
  while (( tries < 3 )); do
    ans="$(ask "请输入雷达 IP: " "")"
    if valid_ipv4 "$ans"; then
      LIDAR_IP="$ans"; SUBNET="$(subnet_of "$ans")"
      return 0
    fi
    warn "不是合法的 IPv4：${ans:-（空）}"
    tries=$((tries + 1))
  done
  return 1
}

choose_host_ip_mode() { # 未发现主机 IP 时选自动/手动；结果放 HOST_IP_MODE
  HOST_IP_MODE=auto
  printf '\n'
  printf '  未发现主机 IP，选择配置方式：\n'
  printf '    1) 自动分配（在 %s.0/24 选一个雷达之外的空闲地址）\n' "$SUBNET"
  printf '    2) 手动输入\n'
  local pick
  pick="$(ask "请选择 [1]: " "1")"
  [[ $pick =~ ^[0-9]+$ ]] || pick=1
  if [[ $pick == 2 ]]; then
    HOST_IP_MODE=manual
  fi
  return 0
}

auto_host_address() { # <iface>：在 SUBNET 内选一个空闲且非雷达地址，结果放 IP_ANSWER
  local iface="$1" local_list candidate octet rc
  local -a candidates=()
  local -A seen=()

  if [[ -z ${SUBNET:-} || -z ${LIDAR_IP:-} ]]; then
    return 1
  fi

  # 雷达 IP 永远排除；常用主机地址优先，再按 2..254 顺序补足。
  local last="${LIDAR_IP##*.}"
  for octet in 5 6 7 49 100 250 251 252 253 254; do
    [[ $octet == "$last" || -n ${seen[$octet]:-} ]] && continue
    seen[$octet]=1; candidates+=("$octet")
  done
  for octet in $(seq 2 254); do
    [[ $octet == "$last" || -n ${seen[$octet]:-} ]] && continue
    candidates+=("$octet")
  done

  local check_arp=true
  if $DRY_RUN; then
    check_arp=false
  elif [[ $(iface_state "$iface") != UP ]]; then
    check_arp=false
    info "网卡 $iface 未启用，自动地址只避开本机地址，未做 ARP 冲突检查"
  elif ! command -v arping >/dev/null 2>&1; then
    warn "缺少 arping（iputils-arping），无法自动检查地址占用"
    return 1
  fi

  local_list=" $(local_ips | tr '\n' ' ') "
  for octet in "${candidates[@]}"; do
    candidate="${SUBNET}.${octet}"
    [[ $local_list == *" $candidate "* ]] && continue
    if $check_arp; then
      rc=0
      run_root arping -D -q -c 2 -w 2 -I "$iface" "$candidate" >/dev/null 2>&1 || rc=$?
      (( rc == 0 )) || {
        (( rc == 1 )) && continue
        warn "ARP 地址检查失败（${candidate}，退出码 ${rc}）"
        return 1
      }
    fi
    IP_ANSWER="$candidate"
    return 0
  done
  return 1
}

ask_host_address() { # 问本机在 SUBNET 的地址；结果放 IP_ANSWER；返回 1 表示跳过
  local def="${HOST_IP_ARG:-}" ans="" tries=0
  while (( tries < 3 )); do
    ans="$(ask "请输入本机 IP${def:+ [默认 ${def}]}: " "$def")"
    if [[ -z $ans ]]; then IP_ANSWER=""; return 1; fi
    if valid_ipv4 "$ans" && on_subnet "$ans"; then IP_ANSWER="$ans"; return 0; fi
    warn "IP 不合法或不在 ${SUBNET}.0/24：$ans"
    tries=$((tries + 1)); def=""
  done
  IP_ANSWER=""
  return 1
}

get_ips() {
  step 3 9 "获取雷达和主机 IP"

  if [[ -f $MIGRATION_STATE ]]; then
    resume_lidar_migration
    return $?
  fi

  # ---- 雷达 IP：先扫描，扫到给候选；扫不到再手动输入 ----
  if [[ -n ${LIDAR_IP_ARG:-} ]]; then
    if ! valid_ipv4 "$LIDAR_IP_ARG"; then
      err "--lidar-ip 不合法：${LIDAR_IP_ARG}"
      ERRORS+=("--lidar-ip 非法")
      return 0
    fi
    LIDAR_IP="$LIDAR_IP_ARG"
    SUBNET="$(subnet_of "$LIDAR_IP")"
    info "雷达 IP（参数）：$LIDAR_IP"
  else
    act "在全部有线接口并行发现雷达（不限制 IP 网段）..."
    local ip
    local -a cand=()
    $DRY_RUN || ensure_sudo
    while IFS= read -r ip; do [[ -n $ip ]] && cand+=("$ip"); done < <(scan_all_radars)
    if (( ${#cand[@]} > 0 )); then
      local c dev
      local pick="${cand[0]}"
      if (( ${#cand[@]} == 1 )); then
        ok "扫描到雷达：$(lidar_label "$pick")"
      else
        info "扫描到 ${#cand[@]} 个设备："
        for c in "${cand[@]}"; do printf '    - %s\n' "$(lidar_label "$c")"; done
        if $ASSUME_YES || [[ ! -t 0 ]]; then
          err "发现多个雷达；请用 --lidar-ip 指定目标，避免自动修改错误设备"
          return 1
        fi
        pick="$(ask "请选择雷达 IP [${cand[0]%%|*}]: " "${cand[0]%%|*}")"
        local picked=""
        for c in "${cand[@]}"; do
          if [[ ${c%%|*} == "$pick" ]]; then picked="$c"; break; fi
        done
        [[ -n $picked ]] || { err "输入的 IP 不在候选列表中，停止部署"; return 1; }
        pick="$picked"
        ok "选择：$(lidar_label "$pick")"
      fi
      LIDAR_IP="${pick%%|*}"
      IFS='|' read -r LIDAR_IP dev LIDAR_SN LIDAR_IFACE <<<"$pick"
      LIDAR_MODEL="$(model_name "$dev")"
      LIDAR_DEV_TYPE="$dev"
      SUBNET="$(subnet_of "$LIDAR_IP")"
    else
      warn "未扫描到雷达"
      if ! ask_radar_ip; then
        SKIPPED+=("雷达 IP（未提供）")
        return 0
      fi
      info "雷达 IP（手动）：$LIDAR_IP"
    fi
  fi

  # ---- 侦测到雷达 IP 后：询问是否经 SDK 修改（需先停止驱动） ----
  if ! maybe_change_lidar_ip; then
    return 1
  fi

  # ---- 雷达网段与 Wi-Fi 网段相同：先询问，再决定是否迁移雷达和有线主机 IP ----
  local wifi_hits=""
  if [[ -n $SUBNET ]]; then
    wifi_hits="$(wifi_on_subnet "$SUBNET" || true)"
  fi
  if [[ -n $wifi_hits ]]; then
    printf '\n'
    warn "雷达网段 ${SUBNET}.0/24 与无线网段相同："
    local wifi_ifc wifi_cidr
    while read -r wifi_ifc wifi_cidr; do
      [[ -n $wifi_ifc ]] && printf '    - %s  %s\n' "$wifi_ifc" "$wifi_cidr"
    done <<<"$wifi_hits"
    info "同一网段同时出现在有线和无线网卡上会造成路由冲突，雷达数据可能走错接口。"
    if $LIDAR_IP_CHANGED; then
      warn "你已手动指定雷达 IP；保留该网段，不再自动迁移。"
      SKIPPED+=("雷达/有线 IP 迁移（用户已手动修改雷达 IP）")
    elif confirm "是否把雷达和有线主机 IP 一起迁移到其它未占用网段？" Y; then
      migrate_lidar_network
      return $?
    else
      warn "保留当前网段；若雷达数据异常，请重新运行并选择迁移。"
      SKIPPED+=("雷达/有线 IP 迁移（用户跳过，与 Wi-Fi 网段冲突）")
    fi
  fi

  # ---- 主机 IP：已有就用；没有就问 ----
  if find_host_net; then
    ok "主机 IP：${HOST_IP}（${HOST_IFACE} ${HOST_CIDR}）"
    return 0
  fi
  if ! choose_wired_iface; then
    warn "没有可用有线网卡，跳过主机 IP"
    SKIPPED+=("主机 IP（没有有线网卡）")
    return 0
  fi

  # 未发现主机 IP：给出数字选项，1 自动分配 / 2 手动输入。
  if [[ -z ${HOST_IP_ARG:-} ]]; then
    choose_host_ip_mode
  else
    HOST_IP_MODE=manual
  fi
  if [[ $HOST_IP_MODE == auto ]]; then
    if auto_host_address "$CHOSEN_IFACE"; then
      ok "自动选择主机 IP：${IP_ANSWER}（雷达 ${LIDAR_IP}）"
      apply_host_address "$CHOSEN_IFACE" "$IP_ANSWER"
      return 0
    fi
    warn "自动选择主机 IP 失败，改为手动输入"
  fi

  if ! ask_host_address; then
    warn "未提供主机 IP，跳过地址配置"
    SKIPPED+=("主机 IP（未提供）")
    return 0
  fi
  apply_host_address "$CHOSEN_IFACE" "$IP_ANSWER"
}

# ----------------------------------------------------------------------------
# 步骤 4：写入 IP 到 Livox 配置
# ----------------------------------------------------------------------------
patch_livox_json() { # <file> <host_ip 可空> <lidar_ip>
  python3 - "$1" "$2" "$3" <<'PY'
import json, sys

path, host_ip, lidar_ip = sys.argv[1], sys.argv[2], sys.argv[3]
try:
    with open(path) as fh:
        data = json.load(fh)
except Exception as exc:                                  # noqa: BLE001
    print(f"ERR 读取失败：{exc}")
    sys.exit(3)

changes = []

def note(where, old, new):
    if old != new:
        changes.append(f"{where}: {old} → {new}")

for block_name, block in data.items():
    if not host_ip or not isinstance(block, dict):
        continue
    hni = block.get("host_net_info")
    if isinstance(hni, list):
        for item in hni:
            if isinstance(item, dict) and item.get("host_ip") is not None:
                note(f"{block_name}.host_net_info.host_ip", item["host_ip"], host_ip)
                item["host_ip"] = host_ip
    elif isinstance(hni, dict):
        for key in ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip"):
            if key in hni:
                note(f"{block_name}.host_net_info.{key}", hni[key], host_ip)
                hni[key] = host_ip

for idx, cfg in enumerate(data.get("lidar_configs", []) or []):
    if isinstance(cfg, dict) and cfg.get("ip"):
        note(f"lidar_configs[{idx}].ip", cfg["ip"], lidar_ip)
        cfg["ip"] = lidar_ip

if changes:
    with open(path, "w") as fh:
        json.dump(data, fh, indent=2, ensure_ascii=False)
        fh.write("\n")
    for c in changes:
        print(f"CHG {c}")
else:
    print("CHG none")
PY
}

apply_config_file() { # <file>
  local f="$1" out rc=0
  if $DRY_RUN; then
    act "[dry-run] 修改 $f（host=${HOST_IP:-不变} lidar=${LIDAR_IP}）"
    return 0
  fi
  if [[ ! -f "${f}.bak" ]]; then
    if [[ -w $f ]]; then cp -a "$f" "${f}.bak"; else run_root cp -a "$f" "${f}.bak"; fi
    info "已备份：${f}.bak"
  fi
  if [[ -w $f ]]; then
    out="$(patch_livox_json "$f" "${HOST_IP:-}" "$LIDAR_IP")" || rc=$?
  else
    local tmp
    tmp="$(mktemp)"
    cp "$f" "$tmp" 2>/dev/null || run_root cp "$f" "$tmp"
    out="$(patch_livox_json "$tmp" "${HOST_IP:-}" "$LIDAR_IP")" || rc=$?
    if (( rc == 0 )); then
      run_root_shell "cat '$tmp' > '$f'"
    fi
    rm -f "$tmp"
  fi
  if (( rc != 0 )); then
    err "写入失败：$f"
    ERRORS+=("Livox 配置写入失败：$f")
    return 0
  fi
  local changed=false
  while IFS= read -r line; do
    [[ -n $line ]] || continue
    case "$line" in
      "CHG none") skip "已经是目标值：$(basename "$f")" ;;
      "CHG "*) ok "${line#CHG }"; changed=true ;;
      "ERR "*) warn "${line#ERR }" ;;
    esac
  done <<<"$out"
  $changed && CONFIG_CHANGED+=("$f")
  return 0
}

write_ip_config() {
  step 4 9 "写入雷达配置（IP + 驱动 launch）"
  local -a files=()
  mapfile -t files < <(livox_config_files)
  if (( ${#files[@]} == 0 )); then
    warn "工作空间里没有找到 Livox 配置（src/livox_ros_driver2/config/）"
    SKIPPED+=("Livox 配置（没有配置文件）")
    return 0
  fi
  if [[ -z ${LIDAR_IP:-} ]]; then
    warn "没有雷达 IP（第 3 步未确定），跳过配置写入"
    SKIPPED+=("Livox 配置写入（没有雷达 IP）")
    return 0
  fi

  act "预览要改动的字段"
  local f
  for f in "${files[@]}"; do
    printf '  %s\n' "$f"
    printf '      host_ip : %s → %s\n' "$(json_field "$f" host)" "${HOST_IP:-（保持不变）}"
    printf '      lidar ip: %s → %s\n' "$(json_field "$f" lidar)" "$LIDAR_IP"
  done

  if ! confirm "是否把以上 IP 写入配置文件？" Y; then
    warn "已跳过配置写入（雷达与主机 IP 不一致时驱动连不上）"
    SKIPPED+=("Livox 配置写入（用户跳过）")
    return 0
  fi

  for f in "${files[@]}"; do
    apply_config_file "$f"
  done
}

# ----------------------------------------------------------------------------
# 步骤 5：获取底盘串口
# ----------------------------------------------------------------------------
collect_ttys() {
  local -a devs=()
  shopt -s nullglob
  devs=(/dev/ttyACM* /dev/ttyUSB*)
  shopt -u nullglob
  ((${#devs[@]})) && printf '%s\n' "${devs[@]}"
  return 0
}

serial_info() {
  local dev="$1" props devpath
  props="$(udevadm info --query=property --name="$dev" 2>/dev/null || true)"
  SERIAL_VID="$(sed -n 's/^ID_VENDOR_ID=//p' <<<"$props")"
  SERIAL_PID="$(sed -n 's/^ID_MODEL_ID=//p' <<<"$props")"
  SERIAL_NO="$(sed -n 's/^ID_SERIAL_SHORT=//p' <<<"$props")"
  SERIAL_MODEL="$(sed -n 's/^ID_MODEL=//p' <<<"$props")"
  devpath="$(sed -n 's/^DEVPATH=//p' <<<"$props")"
  SERIAL_PORT_PATH="$(tr '/' '\n' <<<"$devpath" | grep -E '^[0-9]+-[0-9]+(\.[0-9]+)*(:[0-9]+\.[0-9]+)?$' | tail -n1 | cut -d: -f1)"
}

show_serial() {
  info "USB ID：${SERIAL_VID:-?}:${SERIAL_PID:-?}   型号：${SERIAL_MODEL:-未知}"
  info "序列号：${SERIAL_NO:-(无)}   USB 端口：${SERIAL_PORT_PATH:-(未知)}"
}

get_serial() {
  step 5 9 "获取底盘串口"
  if [[ -n $SERIAL_ARG ]]; then
    if [[ -e $SERIAL_ARG ]]; then
      SERIAL_DEV="$(readlink -f "$SERIAL_ARG")"
      serial_info "$SERIAL_DEV"
      ok "使用指定串口：$SERIAL_ARG → $SERIAL_DEV"
      show_serial
      return 0
    fi
    warn "指定的串口不存在：$SERIAL_ARG，改为自动发现"
  fi

  local -a devs=()
  mapfile -t devs < <(collect_ttys)
  local try=1
  while (( ${#devs[@]} == 0 && try <= 3 )); do
    warn "没有发现 /dev/ttyACM* 或 /dev/ttyUSB*（底盘未插？）"
    if ! confirm "请插好底盘 USB，是否重试（第 ${try}/3 次）？" Y; then break; fi
    sleep 1
    mapfile -t devs < <(collect_ttys)
    try=$((try + 1))
  done

  if (( ${#devs[@]} == 0 )); then
    warn "跳过串口与 udev 配置（可稍后重跑本脚本）"
    SKIPPED+=("串口发现（未插底盘）")
    return 0
  fi

  local dev
  if (( ${#devs[@]} == 1 )); then
    dev="${devs[0]}"
    info "发现唯一串口：$dev"
  else
    info "发现 ${#devs[@]} 个串口："
    local i n=1
    for i in "${devs[@]}"; do printf '    %d) %s\n' "$n" "$i"; n=$((n + 1)); done
    local pick
    pick="$(ask "请选择底盘串口 [1]: " "1")"
    [[ $pick =~ ^[0-9]+$ ]] && (( pick >= 1 && pick <= ${#devs[@]} )) || pick=1
    dev="${devs[$((pick - 1))]}"
  fi

  SERIAL_DEV="$(readlink -f "$dev")"
  serial_info "$SERIAL_DEV"
  ok "串口：$SERIAL_DEV"
  show_serial
}

# ----------------------------------------------------------------------------
# 步骤 6：编写串口 udev 规则
# ----------------------------------------------------------------------------
build_udev_rule() {
  local rule='SUBSYSTEM=="tty", '
  [[ -n $SERIAL_VID ]] && rule+="ATTRS{idVendor}==\"${SERIAL_VID}\", "
  [[ -n $SERIAL_PID ]] && rule+="ATTRS{idProduct}==\"${SERIAL_PID}\", "
  if [[ -n $SERIAL_NO ]]; then
    rule+="ATTRS{serial}==\"${SERIAL_NO}\", "
  elif [[ -n $SERIAL_PORT_PATH ]]; then
    rule+="KERNELS==\"${SERIAL_PORT_PATH}\", "
  fi
  rule+="SYMLINK+=\"${LINK_NAME}\", MODE=\"0666\", GROUP=\"dialout\", TAG+=\"uaccess\""
  printf '%s' "$rule"
}

setup_udev() {
  step 6 9 "编写串口 udev 规则"
  if [[ -z $SERIAL_DEV ]]; then
    skip "没有串口，跳过 udev 规则"
    return 0
  fi
  if [[ -z $SERIAL_VID || -z $SERIAL_PID ]]; then
    warn "读不到 USB VID:PID（udevadm 信息不完整），跳过规则生成"
    SKIPPED+=("udev 规则（串口信息不完整）")
    return 0
  fi

  local rule existing="" same=false
  rule="$(build_udev_rule)"
  [[ -f $UDEV_RULE ]] && existing="$(sed '/^#/d;/^$/d' "$UDEV_RULE" 2>/dev/null || true)"
  [[ $existing == "$rule" ]] && same=true

  printf '\n'
  printf '  规则文件：%s\n' "$UDEV_RULE"
  printf '  规则内容：%s\n' "$rule"
  printf '  固定名称：/dev/%s  →  %s\n' "$LINK_NAME" "$SERIAL_DEV"
  $same && printf '  %s\n' "（文件里已有完全相同的规则）"

  if $same; then
    ok "规则已存在且一致，不重复写入"
  else
    if ! confirm "是否写入 udev 规则并立即生效？" Y; then
      warn "已跳过 udev 规则；launch 里需要手动传 port_name=${SERIAL_DEV}"
      SKIPPED+=("udev 规则写入（用户跳过）")
      return 0
    fi
    if $DRY_RUN; then
      act "[dry-run] 写入 $UDEV_RULE，然后 udevadm reload/trigger"
    else
      ensure_sudo
      local content
      content="# nav_1 底盘串口固定名（install.sh 生成于 $(date '+%F %T')）
$rule"
      printf '%s\n' "$content" | run_root_shell "cat > '$UDEV_RULE'"
      ok "已写入：$UDEV_RULE"
      run_root udevadm control --reload-rules || warn "udevadm reload 失败"
      run_root udevadm trigger --subsystem-match=tty || warn "udevadm trigger 失败"
      run_root udevadm settle --timeout=5 2>/dev/null || true
      local i
      for i in 1 2 3 4 5 6; do
        [[ -e "/dev/$LINK_NAME" ]] && break
        sleep 0.5
      done
    fi
    UDEV_WRITTEN=true
  fi

  if [[ -e "/dev/$LINK_NAME" ]]; then
    ok "固定串口名已生效：/dev/${LINK_NAME} → $(readlink -f "/dev/$LINK_NAME")"
  elif $DRY_RUN; then
    info "dry-run：未真正写入，/dev/${LINK_NAME} 不会出现"
  else
    warn "规则已写好，但 /dev/${LINK_NAME} 还没出现（重新插拔底盘后生效）"
  fi
}

# ----------------------------------------------------------------------------
# 步骤 7：编译
# ----------------------------------------------------------------------------
run_build() {
  step 7 9 "编译工作空间"
  act "加载 ROS 2 Humble 环境"
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
  ok "ROS 环境已加载"

  local -a pkgs=()
  local p d
  for p in "${BUILD_PACKAGES[@]}"; do
    for d in "$WS/src"/*/; do
      [[ -f "$d/package.xml" ]] || continue
      if grep -q "<name>[[:space:]]*${p}[[:space:]]*</name>" "$d/package.xml"; then
        pkgs+=("$p")
        break
      fi
    done
  done

  if (( ${#pkgs[@]} == 0 )); then
    err "src/ 下没有找到要编译的包"
    ERRORS+=("编译未执行（未找到包）")
    BUILD_RESULT="未执行（未找到包）"
    return 0
  fi
  info "将编译 ${#pkgs[@]} 个包：${pkgs[*]}"

  livox_ok || warn "Livox-SDK2 静态库仍缺失，livox_ros_driver2 很可能编译失败"
  gtsam_ok || warn "GTSAM 仍缺失，fastlio_lc_pgo 很可能编译失败"

  local make_jobs pkg_workers
  make_jobs="$(safe_make_jobs)"
  pkg_workers="$JOBS"
  if (( make_jobs < JOBS )); then
    pkg_workers=1
    warn "可用内存有限：make 并行度 ${JOBS} → ${make_jobs}，包并行度 → 1（单个定位组件 TU 峰值约 3 GiB）"
  fi

  if $DRY_RUN; then
    act "[dry-run] MAKEFLAGS=-j${make_jobs} colcon build --symlink-install --packages-select ${pkgs[*]} --parallel-workers ${pkg_workers} --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"
    BUILD_RESULT="dry-run（未编译）"
    return 0
  fi

  act "colcon build（make 并行 ${make_jobs}，包并行 ${pkg_workers}）"
  cd "$WS"
  local rc=0 saved_makeflags="${MAKEFLAGS-}"
  # 显式设置 MAKEFLAGS 后，colcon 不再自行追加 -j$(nproc)；make 会采用该值。
  export MAKEFLAGS="-j${make_jobs}"
  colcon build --symlink-install --packages-select "${pkgs[@]}" \
    --parallel-workers "$pkg_workers" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    --event-handlers console_cohesion+ || rc=$?
  if [[ -n $saved_makeflags ]]; then
    export MAKEFLAGS="$saved_makeflags"
  else
    unset MAKEFLAGS
  fi

  if (( rc == 0 )); then
    ok "编译完成"
    BUILD_RESULT="成功（${#pkgs[@]} 个包）"
  else
    err "编译失败（colcon 退出码 ${rc}）"
    BUILD_RESULT="失败"
    ERRORS+=("编译失败（colcon 退出码 ${rc}）")
    if [[ -n $LOG_FILE ]]; then
      BUILD_FAILED_PKGS="$(grep -oE 'Failed[[:space:]]+<<<[[:space:]]+[A-Za-z0-9_]+' "$LOG_FILE" 2>/dev/null \
        | awk '{print $3}' | sort -u | tr '\n' ' ' || true)"
    fi
    [[ -n $BUILD_FAILED_PKGS ]] || BUILD_FAILED_PKGS="（见 $WS/log/latest_build 与上方 colcon 输出）"
  fi
  return 0
}

# ----------------------------------------------------------------------------
# 步骤 8：配置 DDS（可选）
# ----------------------------------------------------------------------------
DDS_RC_BEGIN="# >>> bw_nav DDS (install.sh) >>>"
DDS_RC_END="# <<< bw_nav DDS (install.sh) <<<"
DDS_RESULT="未修改"

dds_setting_label() { # rmw 包名 → 展示名
  case "$1" in
    rmw_fastrtps_cpp|rmw_fastdds_cpp) printf 'fastdds' ;;
    rmw_cyclonedds_cpp)               printf 'cyclonedds' ;;
    *)                                printf '%s' "$1" ;;
  esac
}

dds_current_setting() { # 当前环境或 shell 配置里生效的 DDS
  local v="${RMW_IMPLEMENTATION:-}" where="当前环境" rc
  if [[ -z $v ]]; then
    for rc in "$HOME/.bashrc" "$HOME/.zshrc"; do
      [[ -f $rc ]] || continue
      v="$(sed -n 's/^[[:space:]]*export[[:space:]]\{1,\}RMW_IMPLEMENTATION=//p' "$rc" 2>/dev/null | tail -n 1)"
      if [[ -n $v ]]; then where="$rc"; break; fi
    done
  fi
  if [[ -z $v ]]; then
    printf '未设置（ROS 默认 fastdds）'
  else
    printf '%s（%s）' "$(dds_setting_label "$v")" "$where"
  fi
}

write_dds_rc() { # write_dds_rc <rcfile> <rmw>：带标记块写入，重复运行只保留一份
  local rc="$1" rmw="$2" tmp
  tmp="$(mktemp)"
  if [[ -f $rc ]]; then
    awk -v b="$DDS_RC_BEGIN" -v e="$DDS_RC_END" '
      $0 == b { in_block = 1; next }
      $0 == e { in_block = 0; next }
      !in_block { print }
    ' "$rc" > "$tmp" || { rm -f "$tmp"; return 1; }
  fi
  {
    printf '%s\n' "$DDS_RC_BEGIN"
    printf 'export RMW_IMPLEMENTATION=%s\n' "$rmw"
    printf '%s\n' "$DDS_RC_END"
  } >> "$tmp" || { rm -f "$tmp"; return 1; }
  cat "$tmp" > "$rc" || { rm -f "$tmp"; return 1; }
  rm -f "$tmp"
}

configure_dds() {
  step 8 9 "配置 DDS（可选）"
  info "当前 DDS：$(dds_current_setting)"

  if ! confirm "是否修改 DDS 实现？" N; then
    info "保持当前 DDS 不变"
    SKIPPED+=("DDS 配置（用户跳过）")
    return 0
  fi

  local ans rmw pkg label
  printf '\n'
  printf '    1) fastdds      rmw_fastrtps_cpp（ROS 2 Humble 默认）\n'
  printf '    2) cyclonedds   rmw_cyclonedds_cpp\n'
  ans="$(ask '选择 DDS [1/2，默认 1]: ' 1)"

  case "$ans" in
    2|cyclone*|Cyclone*|cyc*|Cyc*)
      label=cyclonedds; rmw=rmw_cyclonedds_cpp; pkg=ros-humble-rmw-cyclonedds-cpp ;;
    1|fast*|Fast*)
      label=fastdds; rmw=rmw_fastrtps_cpp; pkg=ros-humble-rmw-fastrtps-cpp ;;
    *)
      warn "无法识别「${ans}」，默认使用 fastdds"
      label=fastdds; rmw=rmw_fastrtps_cpp; pkg=ros-humble-rmw-fastrtps-cpp ;;
  esac

  if pkg_installed "$pkg"; then
    ok "RMW 包已安装：$pkg"
  else
    act "安装 RMW 包：$pkg"
    if ! apt_install "$pkg"; then
      err "安装失败：$pkg"
      ERRORS+=("DDS 依赖安装失败：$pkg")
      DDS_RESULT="失败（$pkg 未安装）"
      return 0
    fi
  fi

  local rc rc_updated=()
  for rc in "$HOME/.bashrc" "$HOME/.zshrc"; do
    [[ -f $rc ]] || continue
    if $DRY_RUN; then
      act "[dry-run] 写入 $rc：export RMW_IMPLEMENTATION=$rmw"
      continue
    fi
    if write_dds_rc "$rc" "$rmw"; then
      rc_updated+=("$rc")
    else
      warn "写入失败：$rc"
    fi
  done

  if $DRY_RUN; then
    DDS_RESULT="dry-run（$label → $rmw，未写入）"
    return 0
  fi
  if (( ${#rc_updated[@]} == 0 )); then
    err "没有可写入的 shell 配置文件（~/.bashrc / ~/.zshrc）"
    ERRORS+=("DDS 配置未写入")
    DDS_RESULT="失败（无 shell 配置文件）"
    return 0
  fi
  ok "已写入 ${rc_updated[*]}：RMW_IMPLEMENTATION=$rmw"
  info "新开终端生效；当前终端可执行 source ${rc_updated[0]}（必要时先 ros2 daemon stop）"
  DDS_RESULT="$label（$rmw）"
}

# ----------------------------------------------------------------------------
# 步骤 9：输出结果
# ----------------------------------------------------------------------------
driver_launch_summary() { # 报告 launch 文件里当前实际使用的驱动
  local f name names=""
  for f in "${LIDAR_DRIVER_LAUNCH_FILES[@]}"; do
    [[ -f $f ]] || continue
    name="$(grep -oE 'msg_MID360s?_launch\.py' "$f" | head -n 1 || true)"
    names+="${name:-未识别}"$'\n'
  done
  if [[ -z $names ]]; then
    printf '文件缺失'
  else
    printf '%s' "$(printf '%s' "$names" | sort -u | paste -sd'/' -)"
  fi
  return 0
}

print_result() {
  step 9 9 "输出结果"
  printf '\n'
  printf '  ==================== 部署结果 ====================\n'
  printf '  依赖检查      : %s\n' "$( (( ${#MISSING_APT[@]} + ${#MISSING_PIP[@]} == 0 )) && echo '通过（无缺失）' || echo "初始缺 ${#MISSING_APT[@]} 个 apt / ${#MISSING_PIP[@]} 个 pip（处理见上）" )"
  printf '  apt 安装      : %s\n' "$( (( APT_INSTALLED > 0 )) && echo "本次 ${APT_INSTALLED} 个包" || echo '无新增' )"
  printf '  pip 安装      : %s\n' "$( (( PIP_INSTALLED > 0 )) && echo "本次 ${PIP_INSTALLED} 个包" || echo '无新增' )"
  printf '  Livox-SDK2    : %s\n' "$( livox_ok && echo '已就绪' || echo '缺失' )"
  printf '  GTSAM         : %s\n' "$( gtsam_ok && echo '已就绪' || echo '缺失（fastlio_lc_pgo 会失败）' )"
  printf '  主机 IP       : %s\n' "${HOST_IP:-未配置}"
  printf '  网卡          : %s\n' "${HOST_IFACE:-未知}"
  printf '  雷达 IP       : %s\n' "${LIDAR_IP:-未探测}"
  printf '  雷达型号      : %s\n' "$( [[ -n $LIDAR_MODEL ]] && printf '%s%s' "$LIDAR_MODEL" "${LIDAR_SN:+（SN $LIDAR_SN）}" || echo '未知（未扫描到 / 手动填写）' )"
  printf '  驱动 launch   : %s\n' "$(driver_launch_summary)"
  printf '  Livox 配置    : %s\n' "$( (( ${#CONFIG_CHANGED[@]} > 0 )) && echo "${#CONFIG_CHANGED[@]} 个文件（已备份 .bak）" || echo '未改动' )"
  printf '  底盘串口      : %s\n' "${SERIAL_DEV:-未发现}"
  printf '  固定串口名    : %s\n' "$( [[ -e "/dev/$LINK_NAME" ]] && echo "/dev/${LINK_NAME} → $(readlink -f "/dev/$LINK_NAME")" || echo "/dev/${LINK_NAME}（未生效）" )"
  printf '  udev 规则     : %s\n' "$( [[ -f $UDEV_RULE ]] && echo "$UDEV_RULE" || echo '未写入' )"
  printf '  编译          : %s\n' "$BUILD_RESULT"
  [[ -n $BUILD_FAILED_PKGS ]] && printf '  失败包        : %s\n' "$BUILD_FAILED_PKGS"
  printf '  DDS 实现      : %s\n' "$DDS_RESULT"
  printf '  日志          : %s\n' "${LOG_FILE:-（无）}"
  printf '  ==================================================\n'

  if (( ${#ERRORS[@]} > 0 )); then
    printf '\n'
    warn "未完成 / 失败项："
    local e
    for e in "${ERRORS[@]}"; do printf '    ✗ %s\n' "$e"; done
  fi
  if (( ${#SKIPPED[@]} > 0 )); then
    printf '\n'
    info "已跳过："
    local s
    for s in "${SKIPPED[@]}"; do printf '    - %s\n' "$s"; done
  fi

  if ! id -nG "$(id -un)" | tr ' ' '\n' | grep -qx dialout; then
    printf '\n'
    warn "当前用户不在 dialout 组：sudo usermod -aG dialout $(id -un)，之后重新登录（或 newgrp dialout）"
  fi
}

# ----------------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------------
init_log() {
  mkdir -p "$WS/log" 2>/dev/null || true
  if [[ -w "$WS/log" ]]; then
    LOG_FILE="$WS/log/install_$(date +%Y%m%d-%H%M%S).log"
    exec > >(tee -a "$LOG_FILE") 2>&1
  fi
}

on_error() {
  local line="$1" code="$2"
  # 子命令 $(...) 里的失败只影响该子命令，不能当成主流程失败（否则会误报 + 重复打印汇总）
  if (( BASH_SUBSHELL > 0 )); then
    [[ ${NAV1_DEBUG:-0} == 1 ]] && err "子命令失败：脚本第 ${line} 行，退出码 ${code}"
    return 0
  fi
  printf '\n'
  err "执行失败：脚本第 ${line} 行，退出码 ${code}"
  [[ -n $LOG_FILE ]] && info "完整日志：${LOG_FILE}"
  print_result 2>/dev/null || true
  exit "$code"
}

main() {
  parse_args "$@"
  init_log
  trap 'on_error "$LINENO" "$?"' ERR
  trap cleanup EXIT

  printf '\n'
  printf '%s\n' "${C_BLD}nav_1 一键部署${C_RST}  ${C_DIM}（${SCRIPT_NAME} · ${SCRIPT_VERSION} · 单文件）${C_RST}"
  info "工作空间：$WS"
  info "时间：$(date '+%F %T')"
  $DRY_RUN && warn "dry-run 模式：只打印动作，不修改系统"
  [[ -n $LOG_FILE ]] && info "日志：$LOG_FILE"

  check_dependencies
  if $CHECK_ONLY; then
    printf '\n'
    if (( ${#MISSING_APT[@]} + ${#MISSING_PIP[@]} == 0 )) && ! $NEED_LIVOX && ! $NEED_GTSAM; then
      ok "依赖检查通过"
      exit 0
    fi
    warn "存在缺失依赖（不带 --check 重新运行即可安装）"
    exit 1
  fi

  install_missing
  get_ips
  write_ip_config
  select_driver_launch
  get_serial
  setup_udev
  run_build
  configure_dds
  print_result

  (( ${#ERRORS[@]} == 0 )) || exit 1
  exit 0
}

main "$@"
