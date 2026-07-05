#!/bin/sh
# smart_camera launcher — 由桌面图标调用
# 二进制自身会杀 Weston，这里只负责启动和恢复

cd /root
/opt/ui/src/apps/smart_camera_bin

# App 退出后恢复桌面
sleep 0.3
/etc/init.d/S49weston restart &
