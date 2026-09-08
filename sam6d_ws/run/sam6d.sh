#!/usr/bin/env bash
set -eo pipefail

unset PYTHONPATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate sam6d
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-72}"

cd "$(dirname "$0")/.."
config="realtime/run_live_split.yaml"
view=false
record=false
full=false
launch_args=()
while (($#)); do
  case "$1" in
    --view) view=true ;;
    --record) record=true ;;
    --record-full-depth) record=true; full=true ;;
    --config) shift; config="${1:?--config requires a YAML path}" ;;
    config:=*) config="${1#config:=}" ;;
    -h|--help)
      echo "usage: $0 [--view] [--record|--record-full-depth] [--config PATH]"
      exit 0
      ;;
    *) launch_args+=("$1") ;;
  esac
  shift
done

if $record; then
  stamp="$(date +%Y%m%d_%H%M%S)"
  run_dir="$PWD/output/live_$stamp"
  if [[ -e "$run_dir" ]]; then
    run_dir="${run_dir}_$$"
  fi
else
  run_dir="$(python - "$config" "$PWD" <<'PY'
import pathlib, sys, yaml
cfg = yaml.safe_load(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")) or {}
path = pathlib.Path((cfg.get("output") or {}).get("dir", "output/live_cam"))
print(path if path.is_absolute() else pathlib.Path(sys.argv[2]) / path)
PY
)"
fi
mkdir -p "$run_dir"
run_config="$run_dir/run_config.yaml"
python - "$config" "$run_config" "$run_dir" "$view" "$record" "$full" <<'PY'
import pathlib, sys, yaml

source, target, output_dir, view, record, full = sys.argv[1:]
cfg = yaml.safe_load(pathlib.Path(source).read_text(encoding="utf-8")) or {}
out = cfg.setdefault("output", {})
out["dir"] = output_dir
out["view"] = view == "true"
rec = out.setdefault("live_recording", {})
rec["enabled"] = record == "true"
rec["depth_policy"] = "full" if full == "true" else "processed"
if rec["enabled"]:
    out["diagnostics"] = True
pathlib.Path(target).write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
PY

exec ros2 launch realtime/launch/sam6d_split.launch.py \
  config:="$run_config" \
  "${launch_args[@]}"
