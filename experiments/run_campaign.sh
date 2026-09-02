#!/usr/bin/env bash
# Guided runner for the compliance-centre campaign.
#
#   ./experiments/run_campaign.sh status
#   ./experiments/run_campaign.sh next
#   ./experiments/run_campaign.sh all
#   ./experiments/run_campaign.sh series S2
#   ./experiments/run_campaign.sh plots
#
# `next` runs exactly one robot trial, the first one still missing. `all` runs
# every remaining trial back to back; `series` restricts that to one prefix.
# Both stop on the first trial that does not archive, and draw the figures for
# everything archived when they finish. run.sh archives the raw logs, the
# effective parameters, the terminal transcript and the provenance for each
# trial.
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

# Trials run back to back. Nothing physical changes between them: each trial
# only rewrites parameters, and the controller returns to the configured
# initial joint pose before it moves. Stopping for an operator between trials
# would only add the reaction time the operator holds were disabled to remove.
cmd_series() {
  local prefix="${1:-}" any=0 id repeat
  while read -r id repeat; do
    if [ -n "$prefix" ]; then
      case "$id" in
        "$prefix"*) ;;
        *) continue ;;
      esac
    fi
    any=1
    # One retry. The controller recovers the robot itself on the next start,
    # so a dropped real-time packet or a cleared reflex costs a trial rather
    # than the campaign. A second failure is not transient and stops the run,
    # because powering on through a real one would only repeat it.
    if ! run_one "$id" "$repeat"; then
      # A trial that aborts has already created its directory, and run.sh
      # refuses to write into one that exists. An archive without a contact
      # report carries nothing, so it is removed before the retry.
      tag="$(printf 'r%02d' "$repeat")"
      if [ -d "$RESULTS/$id/$tag" ] && \
         ! grep -Eq 'CONTACT-ESTABLISHMENT RESULT|SETUP RESULT' \
             "$RESULTS/$id/$tag/terminal.log" 2>/dev/null; then
        echo "discarding the incomplete archive $id/$tag" >&2
        rm -rf "${RESULTS:?}/$id/$tag"
      fi
      echo "retrying: $id r$repeat" >&2
      if ! run_one "$id" "$repeat"; then
        echo "stopped: $id r$repeat did not archive twice" >&2
        return 1
      fi
    fi
  done < <(pending)

  if [ "$any" -eq 0 ]; then
    echo "nothing pending${prefix:+ for series '$prefix'}"
    return 0
  fi

  cmd_plots
}

# Draw every archived trial that has no figure yet.
cmd_plots() {
  local plotter="$HERE/../analysis/plot_contact_establishment_trial.py" csv drawn=0
  if [ ! -f "$plotter" ]; then
    echo "no plotter at $plotter; skipping figures" >&2
    return 0
  fi
  while IFS= read -r csv; do
    local dir
    dir="$(dirname "$csv")"
    if compgen -G "$dir/*_contact_establishment_trial.pdf" > /dev/null; then
      continue
    fi
    python3 "$plotter" "$csv" --out-dir "$dir" > /dev/null 2>&1 && drawn=$((drawn + 1))
  done < <(find "$RESULTS" -name '*_log.csv' 2>/dev/null | sort)
  echo "drew figures for $drawn trial(s)"
}

case "${1:-status}" in
  status) cmd_status ;;
  next)   cmd_next ;;
  all)    cmd_series "" ;;
  plots)  cmd_plots ;;
  series)
    [ $# -ge 2 ] || { echo "usage: $0 series <S1|S2|S3|S4|S5>" >&2; exit 1; }
    cmd_series "$2"
    ;;
  *)
    echo "usage: $0 {status|next|all|series <prefix>|plots}" >&2
    exit 1
    ;;
esac
