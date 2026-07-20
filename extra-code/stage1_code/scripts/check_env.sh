#!/usr/bin/env bash
# Stage 1 ROS 2 development environment diagnostics.
# Usage: bash stage1_code/scripts/check_env.sh [workspace]
#
# 文件说明：Stage 1 开发环境自检脚本。
# 依次检查系统版本、ROS 2 环境变量、构建工具、
# 工作空间和 .bashrc 配置，输出 PASS/WARN/FAIL 汇总。
# 运行方式：bash stage1_code/scripts/check_env.sh [工作空间路径]

# 三个计数器统计检查结果；WS 是待检查的工作空间路径：
# 优先用命令行第 1 个参数，其次环境变量 ROS2_WS，
# 最后默认 ~/ros2_ws
PASS=0
FAIL=0
WARN=0
WS="${1:-${ROS2_WS:-$HOME/ros2_ws}}"

# 三个小工具函数：打印带颜色的结果行并累加对应计数器
# \033[32m 等是终端 ANSI 颜色码（绿=通过 红=失败 黄=警告）
pass() { printf '  \033[32m[PASS]\033[0m %s\n' "$1"; ((PASS += 1)); }
fail() { printf '  \033[31m[FAIL]\033[0m %s\n' "$1"; ((FAIL += 1)); }
warn() { printf '  \033[33m[WARN]\033[0m %s\n' "$1"; ((WARN += 1)); }

printf '%s\n' '=========================================='
printf '%s\n' '  ROS 2 开发环境诊断'
date '+  %Y-%m-%d %H:%M:%S %Z'
printf '%s\n\n' '=========================================='

# 检查 1：操作系统版本（通过 /etc/os-release 的代号判断）
# ROS 2 Jazzy 官方支持 Ubuntu 24.04 (noble)
printf '%s\n' '[系统环境]'
if grep -q 'VERSION_CODENAME=noble' /etc/os-release 2>/dev/null; then
    pass 'Ubuntu 24.04 (Noble)'
elif grep -q 'VERSION_CODENAME=jammy' /etc/os-release 2>/dev/null; then
    warn 'Ubuntu 22.04 (Jammy)：应配套 ROS 2 Humble'
else
    fail '当前系统不是课程支持的 Ubuntu 24.04/22.04'
fi

# 检查 2：是否运行在 WSL 中；若是，进一步确认 GUI 支持
# （RViz 2/Gazebo 等图形界面依赖 WSLg 或 X11）
if grep -qi microsoft /proc/version 2>/dev/null; then
    pass "WSL 环境${WSL_DISTRO_NAME:+: $WSL_DISTRO_NAME}"
    if [ -n "${WAYLAND_DISPLAY:-}" ]; then
        pass '检测到 WSLg/Wayland 图形环境变量'
    elif [ -n "${DISPLAY:-}" ]; then
        warn '检测到 X11 DISPLAY；仍需用 xclock 或 RViz 实际验证显示'
    else
        warn '未检测到 GUI 环境变量，RViz 2/Gazebo GUI 可能无法显示'
    fi
fi
printf '\n'

# 检查 3：ROS_DISTRO 环境变量（由 source setup.bash 设置）
# 未设置说明当前 shell 还没加载 ROS 2 环境
printf '%s\n' '[ROS 2 环境]'
case "${ROS_DISTRO:-}" in
    jazzy) pass 'ROS_DISTRO = jazzy' ;;
    humble) warn 'ROS_DISTRO = humble（课程代码以 Jazzy 为主）' ;;
    '') fail 'ROS_DISTRO 未设置：请 source /opt/ros/jazzy/setup.bash' ;;
    *) warn "ROS_DISTRO = $ROS_DISTRO（不在本课程主要验证范围）" ;;
esac

# 检查 4：ros2 命令行工具，以及课程用到的 turtlesim/rviz2 包
if command -v ros2 >/dev/null 2>&1; then
    pass 'ros2 CLI 可用'
    if ros2 pkg list 2>/dev/null | grep -Fxq turtlesim; then
        pass 'turtlesim 已安装'
    else
        fail "turtlesim 未安装：sudo apt install ros-${ROS_DISTRO:-jazzy}-turtlesim"
    fi
    if ros2 pkg list 2>/dev/null | grep -Fxq rviz2; then
        pass 'rviz2 已安装'
    else
        fail "rviz2 未安装：sudo apt install ros-${ROS_DISTRO:-jazzy}-rviz2"
    fi
else
    fail 'ros2 命令不可用'
fi
printf '\n'

# 检查 5：构建工具链
# colcon 用于编译工作空间，rosdep 用于安装依赖
printf '%s\n' '[构建工具]'
command -v colcon >/dev/null 2>&1 \
    && pass 'colcon 已安装' \
    || fail 'colcon 未安装：sudo apt install python3-colcon-common-extensions'
command -v rosdep >/dev/null 2>&1 \
    && pass 'rosdep 已安装' \
    || fail 'rosdep 未安装：sudo apt install python3-rosdep'
command -v git >/dev/null 2>&1 \
    && pass "git 已安装 ($(git --version 2>/dev/null))" \
    || warn 'git 未安装'
printf '\n'

# 检查 6：工作空间目录结构是否存在、是否已经 colcon build
printf '%s\n' '[工作空间]'
if [ -d "$WS/src" ]; then
    pass "工作空间目录存在: $WS"
    PKG_COUNT="$(find "$WS/src" -name package.xml -type f 2>/dev/null | wc -l)"
    pass "src/ 下有 $PKG_COUNT 个 ROS 包"
else
    warn "工作空间尚未创建: $WS"
fi

if [ -f "$WS/install/setup.bash" ]; then
    pass '工作空间已构建（install/setup.bash 存在）'
else
    warn "工作空间尚未构建：cd '$WS' 后执行 colcon build"
fi
printf '\n'

# 检查 7：.bashrc 是否配置了开机自动 source ROS 2 环境
printf '%s\n' '[Shell 配置]'
if grep -Fq 'source /opt/ros/jazzy/setup.bash' "$HOME/.bashrc" 2>/dev/null; then
    pass '.bashrc 已配置 ROS 2 Jazzy 环境'
elif grep -Fq 'source /opt/ros/humble/setup.bash' "$HOME/.bashrc" 2>/dev/null; then
    warn '.bashrc 配置为 ROS 2 Humble'
else
    warn '.bashrc 未自动加载 ROS 2；手动 source 后仍可正常使用'
fi
printf '\n%s\n' '=========================================='
printf '  诊断完成: %d 通过 / %d 警告 / %d 失败\n' "$PASS" "$WARN" "$FAIL"
printf '%s\n' '=========================================='

# 有 FAIL 项时以非零退出码结束，方便脚本化判断环境是否就绪
if [ "$FAIL" -gt 0 ]; then
    printf '%s\n' '  请修复 FAIL 项后再继续课程。'
    exit 1
fi

printf '%s\n' '  核心环境就绪；WARN 项请结合当前实验需求处理。'
