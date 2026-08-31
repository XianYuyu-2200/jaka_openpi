#!/usr/bin/env bash
set -euo pipefail

filter_mode="${1:-lpf}"
lpf_cutoff="${2:-1.5}"
base_color_sharpness="${JAKA_BASE_COLOR_SHARPNESS:-50}"
wrist_color_sharpness="${JAKA_WRIST_COLOR_SHARPNESS:-75}"
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
teleop_script="${repo_dir}/deployment/jaka_mini2/run_six_joint_teleop.sh"
python_bin="${repo_dir}/.venv/bin/python"
o6_socket="${JAKA_O6_CONTROL_SOCKET:-/tmp/jaka_o6_control.sock}"
record_socket="${JAKA_RECORD_STATE_SOCKET:-/tmp/jaka_teleop_record.sock}"
record_pid=""
teleop_pid=""

case "${filter_mode}" in
    baseline|nlf|lpf) ;;
    *) echo "usage: $0 [baseline|nlf|lpf] [lpf_cutoff]" >&2; exit 64 ;;
esac
if [[ ! -x "${teleop_script}" || ! -x "${python_bin}" ]]; then
    echo "teleoperation script or Python environment is missing" >&2
    exit 66
fi
existing_processes="$({
    pgrep -af '/six_joint_follow_test ' || true
    pgrep -af '/operator_joint_publisher ' || true
    pgrep -af 'python.*deployment\.jaka_mini2\.(control_o6_keyboard_direct|record_teleop_o6)' || true
} | awk -v self="$$" '$1 != self')"
if [[ -n "${existing_processes}" ]]; then
    echo "Another JAKA teleoperation/recording process is already running:" >&2
    echo "${existing_processes}" >&2
    exit 73
fi

cleanup() {
    trap - EXIT INT TERM
    if [[ -n "${record_pid}" ]]; then kill -TERM "${record_pid}" 2>/dev/null || true; fi
    if [[ -n "${teleop_pid}" ]]; then kill -TERM "${teleop_pid}" 2>/dev/null || true; fi
    if [[ -n "${record_pid}" ]]; then wait "${record_pid}" 2>/dev/null || true; fi
    if [[ -n "${teleop_pid}" ]]; then wait "${teleop_pid}" 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM

cd "${repo_dir}"
echo "Starting continuous .101 -> .102 teleoperation with recording (${filter_mode}) ..."
JAKA_O6_CONTROL_SOCKET="${o6_socket}" \
JAKA_RECORD_STATE_SOCKET="${record_socket}" \
JAKA_HOLD_ON_OPERATOR_IDLE=1 \
"${teleop_script}" 0 "${filter_mode}" "${lpf_cutoff}" &
teleop_pid=$!

for _ in {1..50}; do
    if [[ -S "${o6_socket}" ]]; then break; fi
    if ! kill -0 "${teleop_pid}" 2>/dev/null; then wait "${teleop_pid}"; exit $?; fi
    sleep 0.1
done
if [[ ! -S "${o6_socket}" ]]; then
    echo "O6 command socket was not created: ${o6_socket}" >&2
    exit 72
fi

echo "Starting recorder. Controls: r=start, s=success, f=failed, 1-5/Space=O6, q=quit"
echo "Camera sharpness: base=${base_color_sharpness}, wrist=${wrist_color_sharpness}"
JAKA_RECORD_STATE_SOCKET="${record_socket}" \
JAKA_BASE_COLOR_SHARPNESS="${base_color_sharpness}" \
JAKA_WRIST_COLOR_SHARPNESS="${wrist_color_sharpness}" \
"${python_bin}" -m deployment.jaka_mini2.record_teleop_o6 \
    --command-socket "${o6_socket}" \
    --record-socket "${record_socket}" </dev/tty &
record_pid=$!

set +e
wait -n "${teleop_pid}" "${record_pid}"
status=$?
set -e
exit "${status}"
