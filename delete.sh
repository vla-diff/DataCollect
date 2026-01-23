#!/bin/bash

# 设置工作目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"
SAVE_DATA_DIR="$WORKSPACE_DIR/save_data"
rm -rf "$SAVE_DATA_DIR"/*
