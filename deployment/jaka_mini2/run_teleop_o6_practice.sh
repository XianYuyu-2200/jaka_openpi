#!/usr/bin/env bash
set -euo pipefail

filter_mode="${1:-lpf}"
lpf_cutoff="${2:-1.0}"
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
teleop_script="${repo_dir}/deployment/jaka_mini2/run_six_joint_teleop.sh"
python_bin="${repo_dir}/.venv/bin/python"
event_log="${repo_dir}/local_runtime/logs/o6_practice_events.jsonl"
o6_socket="${JAKA_O6_CONTROL_SOCKET:-/tmp/jaka_o6_control.sock}"
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

existing_processes="$({
    pgrep -af '/six_joint_follow_test ' || true
    pgrep -af '/operator_joint_publisher ' || true
    pgrep -af 'python.*deployment\.jaka_mini2\.control_o6_keyboard_direct' || true
} | awk -v self="$$" '$1 != self')"
if [[ -n "${existing_processes}" ]]; then
    echo "Another JAKA teleoperation/O6 process is already running:" >&2
    echo "${existing_processes}" >&2
    echo "Stop it before starting a new combined practice session." >&2
    exit 73
fi

cleanup() {
    trap - EXIT INT TERM
    if [[ -n "${hand_pid}" ]]; then
        kill -TERM "${hand_pid}" 2>/dev/null || true
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
JAKA_O6_CONTROL_SOCKET="${o6_socket}" \
JAKA_HOLD_ON_OPERATOR_IDLE=1 \
"${teleop_script}" 0 "${filter_mode}" "${lpf_cutoff}" &
teleop_pid=$!

sleep 2
if ! kill -0 "${teleop_pid}" 2>/dev/null; then
    wait "${teleop_pid}"
    exit $?
fi
for _ in {1..30}; do
    [[ -S "${o6_socket}" ]] && break
    sleep 0.1
done
if [[ ! -S "${o6_socket}" ]]; then
    echo "O6 command socket was not created: ${o6_socket}" >&2
    exit 72
fi

echo "Practice controls: 1-5 O6 presets; q quits hand control and both arms."
echo "O6 event log: ${event_log}"
"${python_bin}" -m deployment.jaka_mini2.control_o6_keyboard_direct \
    --event-log "${event_log}" \
    --command-socket "${o6_socket}" </dev/tty &
hand_pid=$!

set +e
wait -n "${teleop_pid}" "${hand_pid}"
status=$?
set -e
exit "${status}"
