#!/usr/bin/env bash
set -euo pipefail

duration_sec="${1:-30}"
filter_mode="${2:-baseline}"
lpf_cutoff="${3:-1.0}"
build_dir="${JAKA_TELEOP_BUILD_DIR:-/tmp/jaka_mini2_adapter-hardware}"
socket_path="${JAKA_TELEOP_SOCKET:-/tmp/jaka_six_joint.sock}"
follower_bin="${build_dir}/six_joint_follow_test"
publisher_bin="${build_dir}/operator_joint_publisher"
# Use separate performance cores by default (avoid E-cores 12-15 and the
# enp4s0 IRQ currently assigned to CPU 4).
follower_cpu="${JAKA_FOLLOWER_CPU:-8}"
publisher_cpu="${JAKA_PUBLISHER_CPU:-10}"
sampler_cpu="${JAKA_SAMPLER_CPU:-6}"
follower_pid=""
publisher_pid=""

if [[ ! "${duration_sec}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "usage: $0 [duration_sec] [baseline|nlf|lpf] [lpf_cutoff]" >&2
    exit 64
fi
case "${filter_mode}" in
    baseline|nlf|lpf) ;;
    *) echo "filter must be baseline, nlf or lpf" >&2; exit 64 ;;
esac
if [[ "${filter_mode}" == "lpf" &&
      ! "${lpf_cutoff}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "lpf_cutoff must be a positive number" >&2
    exit 64
fi
if [[ ! -x "${follower_bin}" || ! -x "${publisher_bin}" ]]; then
    echo "teleoperation binaries not found in ${build_dir}" >&2
    echo "build the hardware adapter first; see deployment/jaka_mini2/README.md" >&2
    exit 66
fi
for cpu in "${follower_cpu}" "${publisher_cpu}" "${sampler_cpu}"; do
    if [[ ! "${cpu}" =~ ^[0-9]+$ ]] || (( cpu >= $(nproc) )); then
        echo "invalid teleoperation CPU core: ${cpu}" >&2
        exit 64
    fi
done

cleanup() {
    trap - EXIT INT TERM
    if [[ -n "${publisher_pid}" ]]; then
        kill -INT "${publisher_pid}" 2>/dev/null || true
    fi
    if [[ -n "${follower_pid}" ]]; then
        kill -INT "${follower_pid}" 2>/dev/null || true
    fi
    if [[ -n "${publisher_pid}" ]]; then
        wait "${publisher_pid}" 2>/dev/null || true
    fi
    if [[ -n "${follower_pid}" ]]; then
        wait "${follower_pid}" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

echo "Starting .102 follower for ${duration_sec}s ..."
JAKA_SERVO_FILTER="${filter_mode}" JAKA_LPF_CUTOFF="${lpf_cutoff}" \
taskset -c "${follower_cpu}" "${follower_bin}" \
    192.168.0.102 "${socket_path}" START_SIX_JOINTS "${duration_sec}" &
follower_pid=$!

# Give the follower time to log in and bind the Unix datagram socket.
sleep 1
if ! kill -0 "${follower_pid}" 2>/dev/null; then
    wait "${follower_pid}"
    exit $?
fi

echo "Starting .101 operator publisher; press Ctrl+C to stop both ..."
JAKA_PUBLISHER_CPU="${publisher_cpu}" JAKA_SAMPLER_CPU="${sampler_cpu}" \
taskset -c "${publisher_cpu},${sampler_cpu}" "${publisher_bin}" \
    192.168.0.101 "${socket_path}" "${duration_sec}" &
publisher_pid=$!

set +e
wait -n "${follower_pid}" "${publisher_pid}"
status=$?
set -e
exit "${status}"
