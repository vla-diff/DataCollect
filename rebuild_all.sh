#!/usr/bin/env bash
set -euo pipefail

workspaces=(
  "EGO-Planner-v3"
  "Fast-Perching"
  "lidar_filter"
  "ROS-Unity_bridge"
)

clean_cache=false
if [[ "${1-}" == "--clean-cache" ]]; then
  clean_cache=true
  shift
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v catkin_make >/dev/null 2>&1; then
  echo "Error: catkin_make not found in PATH. Source your ROS setup first." >&2
  exit 1
fi

for ws in "${workspaces[@]}"; do
  ws_path="${script_dir}/${ws}"
  if [[ ! -d "${ws_path}" ]]; then
    echo "Error: workspace not found: ${ws_path}" >&2
    exit 1
  fi

  cache_path="${ws_path}/build/CMakeCache.txt"
  if [[ -f "${cache_path}" ]]; then
    if command -v rg >/dev/null 2>&1; then
      cached_src="$(rg -m 1 -o "CMAKE_HOME_DIRECTORY:INTERNAL=.*" "${cache_path}" | cut -d= -f2-)"
    else
      cached_src="$(grep -m 1 "CMAKE_HOME_DIRECTORY:INTERNAL=" "${cache_path}" | cut -d= -f2-)"
    fi
    expected_src="${ws_path}/src"
    if [[ -n "${cached_src}" && "${cached_src}" != "${expected_src}" ]]; then
      if [[ "${clean_cache}" == true ]]; then
        echo "==> Cleaning cache for ${ws_path} (source changed: ${cached_src} -> ${expected_src})"
        rm -rf "${ws_path}/build" "${ws_path}/devel"
      else
        echo "Error: CMake cache points to a different source directory:" >&2
        echo "  cached:  ${cached_src}" >&2
        echo "  current: ${expected_src}" >&2
        echo "Hint: rerun with --clean-cache to reset build/devel for this workspace." >&2
        exit 1
      fi
    fi
  fi

  echo "==> Building ${ws_path}"
  (cd "${ws_path}" && catkin_make "$@")
done

echo "All workspaces built successfully."
