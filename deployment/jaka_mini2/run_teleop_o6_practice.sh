#!/usr/bin/env bash
set -euo pipefail

filter_mode="${1:-lpf}"
lpf_cutoff="${2:-1.0}"
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
teleop_script="${repo_dir}/deployment/jaka_mini2/run_six_joint_teleop.sh"
python_bin="${repo_dir}/.venv/bin/python"
event_log="${repo_dir}/local_runtime/logs/o6_practice_events.jsonl"
teleop_pid=""
hand_pid=""

case "${filter_mode}" in
    baseline|nlf|lpf) ;;
    *) echo "usage: $0 [baseline|nlf|lpf] [lpf_cutoff]" >&2; exit 64 ;;
esac
if [[ ! -x "${teleop_script}" || ! -x "${python_bin}" ]]; then
    echo "teleoperation script or Python environment is missing" >&2
    exit 66
fi

cleanup() {
    trap - EXIT INT TERM
    if [[ -n "${hand_pid}" ]]; then
        kill -INT "${hand_pid}" 2>/dev/null || true
    fi
    if [[ -n "${teleop_pid}" ]]; then
        kill -TERM "${teleop_pid}" 2>/dev/null || true
    fi
    if [[ -n "${hand_pid}" ]]; then
        wait "${hand_pid}" 2>/dev/null || true
    fi
    if [[ -n "${teleop_pid}" ]]; then
        wait "${teleop_pid}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "Starting continuous .101 -> .102 teleoperation (${filter_mode}) ..."
"${teleop_script}" 0 "${filter_mode}" "${lpf_cutoff}" &
teleop_pid=$!

sleep 2
if ! kill -0 "${teleop_pid}" 2>/dev/null; then
    wait "${teleop_pid}"
    exit $?
fi

echo "Practice controls: 1-5 O6 presets; q quits hand control and both arms."
echo "O6 event log: ${event_log}"
"${python_bin}" -m deployment.jaka_mini2.control_o6_keyboard_direct \
    --event-log "${event_log}" </dev/tty &
hand_pid=$!

set +e
wait -n "${teleop_pid}" "${hand_pid}"
status=$?
set -e
exit "${status}"
