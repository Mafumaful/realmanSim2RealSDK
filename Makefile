# 三臂机器人控制 Makefile
# 使用方法: make build && make run

SHELL := /bin/bash
PKG_DIR := $(shell pwd)/src
WS_DIR := $(shell pwd)/ws

.PHONY: all build run clean help

help:
	@echo "可用命令:"
	@echo "  make build  - 构建ROS2功能包"
	@echo "  make run    - 启动关节控制GUI"
	@echo "  make clean  - 清理构建文件"

build:
	@mkdir -p $(WS_DIR)/src
	@ln -sf $(PKG_DIR) $(WS_DIR)/src/triarm_control 2>/dev/null || true
	@cd $(WS_DIR) && source /opt/ros/humble/setup.bash && colcon build --packages-select triarm_control
	@echo "构建完成"

run:
	@cd $(WS_DIR) && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch triarm_control triarm_control.launch.py

clean:
	@rm -rf $(WS_DIR)/build $(WS_DIR)/install $(WS_DIR)/log
	@echo "清理完成"

all: build run
