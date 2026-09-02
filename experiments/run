#!/usr/bin/env bash
# Run one experiment setup and archive its logs with full provenance.
#
#   experiments/run.sh <run_id> [repeat_index]
#   experiments/run.sh S2_t1_10deg 1
#
# What it does, in order:
#   1. Saves the current params/ directory so experimental overlays never
#      contaminate the next run.
#   2. Applies the setup's overlay onto params/.
#   3. Runs the controller. You drive it interactively as usual, or through
#      lib/auto_drive.py.
#   4. Copies the CSV logs plus the effective parameters, the commit and the
#      terminal transcript into experiments/results/<run_id>/rNN/.
#   5. Restores the original params/ even if the run crashed or was aborted.
#
# The restore is in a trap, so Ctrl-C and libfranka reflex exits are safe.
#
# The surface plane is calibrated once and held for the whole campaign, so no
# per-setup plane or tool profile is applied here. The active values are
# archived with every trial in params_effective/.

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
SGC="$REPO/surface_grinding_controller"
PARAMS="$SGC/params"
BINARY="$SGC/surface_grinding_controller"

if [ $# -lt 1 ]; then
  echo "usage: $(basename "$0") <run_id> [repeat_index]" >&2
  echo "available setups:" >&2
  ls -1 "$HERE/setups" 2>/dev/null | grep -v INDEX.txt | sed 's/^/  /' >&2
  exit 1
fi

RUN_ID="$1"
REPEAT="${2:-1}"
SETUP_DIR="$HERE/setups/$RUN_ID"
OVERLAY="$SETUP_DIR/overlay.txt"

if [ ! -f "$OVERLAY" ]; then
  echo "ERROR: no setup '$RUN_ID' (missing $OVERLAY)" >&2
  exit 1
fi

if [ ! -x "$BINARY" ]; then
  echo "ERROR: $BINARY is not built. Run make in $SGC first." >&2
  exit 1
fi

REPEAT_TAG="$(printf 'r%02d' "$REPEAT")"
OUT="$HERE/results/$RUN_ID/$REPEAT_TAG"

if [ -d "$OUT" ]; then
  echo "ERROR: $OUT already exists. Use a different repeat index, or delete it." >&2
  exit 1
fi

# Sampled BEFORE the overlay is applied. The overlay deliberately edits tracked
# files in params/, so sampling after it would report every trial as dirty.
# The overlay is recorded verbatim in overlay.txt and params_effective/; what
# this field records is whether anything ELSE was uncommitted at run time.
COMMIT="$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo unknown)"
if [ -n "$(git -C "$REPO" status --porcelain 2>/dev/null)" ]; then
  TREE_STATE=dirty
else
  TREE_STATE=clean
fi

BACKUP="$(mktemp -d)"
cp -a "$PARAMS/." "$BACKUP/"

restore_params() {
  cp -a "$BACKUP/." "$PARAMS/"
  rm -rf "$BACKUP"
}
trap restore_params EXIT

echo "== $RUN_ID $REPEAT_TAG =="
python3 "$HERE/lib/apply_overlay.py" "$OVERLAY" "$PARAMS" || exit 1

mkdir -p "$OUT"
TRANSCRIPT="$OUT/terminal.log"

# The controller writes its CSV paths relative to its own directory.
#
# stdbuf turns the buffering off entirely. Writing to a pipe, C stdio buffers
# in 4 kB blocks, so a prompt that does not fill a block never reaches the
# reader while the controller waits for the answer to it. Driven by
# lib/auto_drive.py that deadlocks until the trial timeout: the reader waits
# for a prompt sitting in the writer's buffer, and the writer waits for input.
#
# Line buffering is not enough. Every prompt here ends in ": " with no newline,
# which is exactly what a line-buffered stream holds back.
( cd "$SGC" && stdbuf -o0 -e0 "$BINARY" ) 2>&1 | tee "$TRANSCRIPT"
STATUS="${PIPESTATUS[0]}"

# Archiving happens whatever the exit status: an aborted trial is still
# evidence, and the transcript says why it stopped.
cp -a "$PARAMS/." "$OUT/params_effective/" 2>/dev/null || {
  mkdir -p "$OUT/params_effective" && cp -a "$PARAMS/." "$OUT/params_effective/"
}
cp "$OVERLAY" "$OUT/overlay.txt"
[ -f "$SETUP_DIR/about.txt" ] && cp "$SETUP_DIR/about.txt" "$OUT/about.txt"

LOGS="$SGC/logs"
if [ -d "$LOGS" ]; then
  mkdir -p "$OUT/logs"
  find "$LOGS" -maxdepth 1 -name '*.csv' -exec mv {} "$OUT/logs/" \;
fi

cat > "$OUT/provenance.txt" <<EOF
run_id:        $RUN_ID
repeat:        $REPEAT_TAG
recorded:      $(date -Iseconds)
commit:        $COMMIT
tree_state:    $TREE_STATE
exit_status:   $STATUS
EOF

echo "archived to $OUT"
exit "$STATUS"
