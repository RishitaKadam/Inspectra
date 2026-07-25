#!/bin/bash
# Generates the Inspectra Panda URDF: runs xacro, then renames fer_* -> panda_*
# so link/joint names match moveit_resources_panda_moveit_config's SRDF.
# Isolated in a script (not inline in the launch file) to avoid Command()'s
# argv-splitting mangling pipe/quote syntax.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
xacro "$SCRIPT_DIR/inspectra_panda.urdf.xacro" | sed 's/fer_/panda_/g'
