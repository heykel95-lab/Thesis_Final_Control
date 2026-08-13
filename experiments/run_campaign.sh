#!/usr/bin/env bash
# Guided runner for the compliance-centre campaign.
#
#   ./experiments/run_campaign.sh status
#   ./experiments/run_campaign.sh next
#   ./experiments/run_campaign.sh series S2
#
# `next` runs exactly one robot trial, the first one still missing. `series`
# runs every remaining trial of one series, stopping on the first trial that
# does not archive. run.sh archives the raw logs, the effective parameters,
# the terminal transcript and the provenance for each one.
#
# Run the series in order. S1 is the reference the others are read against,
# S3 assumes S2's lever, and S4 and S5 sweep the position that S2 established
# is worth having at all.

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUPS="$HERE/setups"
RESULTS="$HERE/results"
REPEATS=3

if [ ! -f "$SETUPS/INDEX.txt" ]; then
  echo "ERROR: no setups. Run lib/generate_setups.py first." >&2
  exit 1
fi

mapfile -t RUN_IDS < <(grep -v '^#' "$SETUPS/INDEX.txt" | grep -v '^$')

# Trials in campaign order: every repeat of a setup before the next setup, and
# the series in their dependency order.
pending() {
  local id repeat tag
  for id in "${RUN_IDS[@]}"; do
    for repeat in $(seq 1 "$REPEATS"); do
      tag="$(printf 'r%02d' "$repeat")"
      if [ ! -d "$RESULTS/$id/$tag" ]; then
        echo "$id $repeat"
      fi
    done
  done
}

cmd_status() {
  local total done_count
  total=$(( ${#RUN_IDS[@]} * REPEATS ))
  done_count=$(( total - $(pending | wc -l) ))
  echo "campaign: $done_count / $total trials archived"
  echo
  local id repeat tag line
  for id in "${RUN_IDS[@]}"; do
    line="  $id  "
    for repeat in $(seq 1 "$REPEATS"); do
      tag="$(printf 'r%02d' "$repeat")"
      if [ -d "$RESULTS/$id/$tag" ]; then line+="[x]"; else line+="[ ]"; fi
    done
    echo "$line"
  done
}

run_one() {
  local id="$1" repeat="$2"
  echo
  echo "================================================================"
  echo "  $id  r$(printf '%02d' "$repeat")"
  echo "================================================================"
  python3 "$HERE/lib/auto_drive.py" "$id" "$repeat"
}

cmd_next() {
  local first
  first="$(pending | head -1)"
  if [ -z "$first" ]; then
    echo "campaign complete"
    return 0
  fi
  # shellcheck disable=SC2086
  run_one $first
}

cmd_series() {
  local prefix="$1" any=0 id repeat
  while read -r id repeat; do
    case "$id" in
      "$prefix"*) ;;
      *) continue ;;
    esac
    any=1
    if ! run_one "$id" "$repeat"; then
      echo "stopped: $id r$repeat did not archive" >&2
      return 1
    fi
    echo "Reset the setup, then press Enter for the next trial."
    read -r _
  done < <(pending)

  if [ "$any" -eq 0 ]; then
    echo "nothing pending for series '$prefix'"
  fi
}

case "${1:-status}" in
  status) cmd_status ;;
  next)   cmd_next ;;
  series)
    [ $# -ge 2 ] || { echo "usage: $0 series <S1|S2|S3|S4|S5>" >&2; exit 1; }
    cmd_series "$2"
    ;;
  *)
    echo "usage: $0 {status|next|series <prefix>}" >&2
    exit 1
    ;;
esac
