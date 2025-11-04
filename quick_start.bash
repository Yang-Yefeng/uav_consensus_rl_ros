#!/bin/bash

# 启动 terminator，并在后台运行（避免阻塞脚本）
terminator &
# 等待 terminator 启动完成（根据系统速度调整等待时间）
sleep 1

# 向 terminator 窗口发送快捷键，分割出 4 个窗口（2×2）
# 依赖 xdotool 工具发送键盘事件，需先安装
xdotool search --name "Terminator" windowactivate --sync
# 1. 水平分割（上下各1个窗口）：Ctrl+Shift+O
xdotool key ctrl+shift+o
# 2. 切换到下方窗口：Alt+Down（移动焦点）
xdotool key alt+Down
# 3. 垂直分割下方窗口（左右各1个）：Ctrl+Shift+E
xdotool key ctrl+shift+e
# 4. 切换到上方窗口：Alt+Up
xdotool key alt+Up
# 5. 垂直分割上方窗口（左右各1个）：Ctrl+Shift+E
xdotool key ctrl+shift+e
