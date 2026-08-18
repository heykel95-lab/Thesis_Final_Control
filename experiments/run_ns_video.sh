#!/usr/bin/env bash
# Run the three null-space video trials of the disturbance case back to back.
#
#   ./experiments/run_ns_video.sh            # next free repeat of each
#   ./experiments/run_ns_video.sh 2          # force repeat index r02
#
# Order matters: mode 0 is the reference the other two are watched against, so
# it is filmed first and the three are filmed in one sitting, with the camera
# and the arm untouched between them.
#
# Nothing physical changes between trials. Each one only rewrites parameters,
# and the controller drives the arm back to the configured initial posture
# before it applies any torque, so the runs chain without an operator. Every
# trial holds for experiment_duration and the controller then exits on its own,
# which is what ends one trial and releases the robot for the next.
#
# Each trial is driven by lib/auto_drive.py, which answers the startup menu
# with h and the null-space prompt with the mode from the overlay. The set-up
# gates never appear in a plain hold, so nothing else is waiting on input.

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULTS="$HERE/results"

RUN_IDS=(V_ns0_baseline V_ns1_damping_20p0 V_ns2_ksigma_2p0 V_ns2_ksigma_return)
FORCED_REPEAT="${1:-}"

# The first repeat index this setup has no archive for, so a second filming
# session lands beside the first instead of refusing to start.
next_repeat() {
  local id="$1" repeat=1
  while [ -d "$RESULTS/$id/$(printf 'r%02d' "$repeat")" ]; do
    repeat=$((repeat + 1))
  done
  echo "$repeat"
}

# An aborted trial still creates its directory, and run.sh refuses to write
# into one that exists. An archive with no CSV recorded nothing, so it is
# removed before the retry rather than blocking it.
discard_if_empty() {
  local dir="$1"
  if [ -d "$dir" ] && ! compgen -G "$dir/logs/*.csv" > /dev/null; then
    echo "discarding the incomplete archive $dir" >&2
    rm -rf "$dir"
  fi
}

# The return trial needs a command sent partway through the run, which the
# prompt-matching driver has no way to express, so it brings its own.
driver_for() {
  case "$1" in
    V_ns2_ksigma_return) echo "$HERE/lib/drive_ksigma_return.py" ;;
    *)                   echo "$HERE/lib/auto_drive.py" ;;
  esac
}

run_one() {
  local id="$1" repeat="$2"
  echo
  echo "================================================================"
  echo "  $id  r$(printf '%02d' "$repeat")"
  echo "================================================================"
  python3 "$(driver_for "$id")" "$id" "$repeat"
}

for id in "${RUN_IDS[@]}"; do
  if [ -n "$FORCED_REPEAT" ]; then
    repeat="$FORCED_REPEAT"
  else
    repeat="$(next_repeat "$id")"
  fi
  out="$RESULTS/$id/$(printf 'r%02d' "$repeat")"

  # One retry. The controller recovers the robot itself on the next start, so
  # a dropped real-time packet or a cleared reflex costs a trial rather than
  # the sitting. A second failure is not transient: stopping leaves the arm
  # where it can be looked at instead of driving it again blind.
  if ! run_one "$id" "$repeat"; then
    discard_if_empty "$out"
    echo "retrying: $id r$repeat" >&2
    if ! run_one "$id" "$repeat"; then
      echo "stopped: $id r$repeat did not archive twice" >&2
      exit 1
    fi
  fi
done

echo
echo "all three trials archived under $RESULTS/V_ns*"
