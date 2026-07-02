#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$SCRIPT_DIR"

INSTALL_CONFIG="$WORKSPACE/install/odin_ros_driver/share/odin_ros_driver/config/control_command.yaml"
SOURCE_CONFIG="$WORKSPACE/src/odin_ros_driver/config/control_command.yaml"
SETUP_FILE="$WORKSPACE/install/setup.bash"

# Edit these presets when you want to change the quick relocalization choices.
PRESET_MAP_1_NAME="Red"
PRESET_MAP_1_PATH="$WORKSPACE/map/Red"
PRESET_MAP_1_INIT_POS="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]"

PRESET_MAP_2_NAME="Blue"
PRESET_MAP_2_PATH="$WORKSPACE/map/Blue"
PRESET_MAP_2_INIT_POS="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]"

require_file() {
  local path="$1"
  if [[ ! -f "$path" ]]; then
    echo "Error: file not found: $path" >&2
    exit 1
  fi
}

require_file "$INSTALL_CONFIG"
require_file "$SETUP_FILE"

echo "请选择启动模式:"
echo "  0) Odometry 模式"
echo "  1) SLAM 建图模式"
echo "  2) Relocalization 重定位模式"
read -r -p "请输入 0/1/2: " MODE

MAP_PATH=""
INIT_POS="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]"

case "$MODE" in
  0)
    echo "已选择 Odometry 模式"
    ;;
  1)
    echo "已选择 SLAM 建图模式"
    ;;
  2)
    echo "请选择重定位地图:"
    echo "  1) $PRESET_MAP_1_NAME ($PRESET_MAP_1_PATH)"
    echo "  2) $PRESET_MAP_2_NAME ($PRESET_MAP_2_PATH)"
    read -r -p "请输入 1/2: " MAP_CHOICE

    case "$MAP_CHOICE" in
      1)
        MAP_PATH="$PRESET_MAP_1_PATH"
        INIT_POS="$PRESET_MAP_1_INIT_POS"
        ;;
      2)
        MAP_PATH="$PRESET_MAP_2_PATH"
        INIT_POS="$PRESET_MAP_2_INIT_POS"
        ;;
      *)
        echo "Error: invalid map choice: $MAP_CHOICE" >&2
        exit 1
        ;;
    esac

    require_file "$MAP_PATH"
    echo "已选择重定位地图: $MAP_PATH"
    ;;
  *)
    echo "Error: invalid mode: $MODE" >&2
    exit 1
    ;;
esac

export ODIN_MODE="$MODE"
export ODIN_MAP_PATH="$MAP_PATH"
export ODIN_INIT_POS="$INIT_POS"

update_config() {
  local config_path="$1"

  python3 - "$config_path" <<'PY'
import os
import re
import sys

config_path = sys.argv[1]
mode = int(os.environ["ODIN_MODE"])
map_path = os.environ["ODIN_MAP_PATH"]
init_pos = os.environ["ODIN_INIT_POS"]

with open(config_path, "r", encoding="utf-8") as f:
    lines = f.readlines()

def yaml_string(value):
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'

def replace_key(lines, key, value):
    pattern = re.compile(rf"^(\s*{re.escape(key)}\s*:\s*)(.*?)(\s+#.*)?(\n?)$")
    for index, line in enumerate(lines):
        match = pattern.match(line)
        if not match:
            continue

        prefix, _old_value, comment, newline = match.groups()
        comment = comment or ""
        newline = newline or "\n"
        spacing = " " if comment and not value.endswith(" ") else ""
        lines[index] = f"{prefix}{value}{spacing}{comment}{newline}"
        return

    raise SystemExit(f"key not found in {config_path}: {key}")

if mode == 2:
    relocalization_map = yaml_string(map_path)
else:
    relocalization_map = '""'
    init_pos = "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]"

replace_key(lines, "custom_map_mode", str(mode))
replace_key(lines, "relocalization_map_abs_path", relocalization_map)
replace_key(lines, "custom_init_pos", init_pos)

with open(config_path, "w", encoding="utf-8") as f:
    f.writelines(lines)
PY
}

update_config "$INSTALL_CONFIG"
echo "已更新: $INSTALL_CONFIG"

if [[ -f "$SOURCE_CONFIG" ]]; then
  update_config "$SOURCE_CONFIG"
  echo "已同步: $SOURCE_CONFIG"
fi

echo "正在启动 odin_ros_driver..."
cd "$WORKSPACE"
set +u
source "$SETUP_FILE"
set -u
exec ros2 launch odin_ros_driver odin1_ros2.launch.py
