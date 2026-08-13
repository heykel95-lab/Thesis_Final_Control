#!/usr/bin/env bash
# Probe the t2 lever magnitude, adopt the best one, then run the campaign.
#
#   ./experiments/run_t2_then_campaign.sh
#
# A 60 mm lever about t2 overshoots: it turns the tool through zero and out to
# +15 deg. The magnitude that suits t1 does not suit t2, because the contact
# face is 20 mm half-width across t2 against 60 mm across t1 and the tool rocks
# over the short dimension. This runs one trial at each candidate magnitude,
# takes the one whose t2 alignment component ends nearest zero, writes it into
# the generator, and continues with the rest of the campaign.
#
# It stops rather than continuing if no candidate improves on doing nothing.
# Running forty trials with a lever that makes the alignment worse would waste
# the campaign, and the guard costs nothing when the probe succeeds.

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULTS="$HERE/results"
GEN="$HERE/lib/generate_setups.py"

CANDIDATES=(010 020 030)

echo "== probing the t2 lever magnitude =="
for mm in "${CANDIDATES[@]}"; do
  id="P_t2mag_${mm}mm"
  if [ -d "$RESULTS/$id/r01" ]; then
    echo "-- $id already recorded"
    continue
  fi
  echo "-- $id"
  python3 "$HERE/lib/auto_drive.py" "$id" 1 || {
    echo "ERROR: $id did not complete" >&2
    exit 1
  }
done

# Read the t2 component the set-up ended on, for each candidate and for the
# no-lever reference at the same commanded offset.
BEST="$(python3 - "$RESULTS" "${CANDIDATES[@]}" <<'EOF'
import os
import re
import sys

results, candidates = sys.argv[1], sys.argv[2:]
pattern = re.compile(r"alignment components.*after=\[\s*([-+\d.]+),\s*([-+\d.]+)")


def after_t2(run_id, repeat="r01"):
    path = os.path.join(results, run_id, repeat, "terminal.log")
    if not os.path.isfile(path):
        return None
    match = pattern.search(open(path, errors="replace").read())
    return float(match.group(2)) if match else None


reference = after_t2("S1_none_t2_05deg")
scored = []
for mm in candidates:
    value = after_t2(f"P_t2mag_{mm}mm")
    if value is not None:
        scored.append((abs(value), mm, value))
        print(f"{mm} mm -> t2 = {value:+.2f} deg", file=sys.stderr)

if reference is not None:
    print(f"no lever -> t2 = {reference:+.2f} deg", file=sys.stderr)

if not scored:
    print("NONE")
    raise SystemExit
scored.sort()
best_abs, best_mm, best_value = scored[0]
if reference is not None and best_abs >= abs(reference):
    print(f"no candidate beat the no-lever reference", file=sys.stderr)
    print("NONE")
else:
    print(best_mm)
EOF
)"

if [ "$BEST" = "NONE" ]; then
  echo "stopping: the probe did not find a magnitude worth running" >&2
  exit 1
fi

echo "== adopting ${BEST} mm for t2 =="
python3 - "$GEN" "$BEST" <<'EOF'
import re
import sys

path, mm = sys.argv[1], int(sys.argv[2])
text = open(path).read()
marker = "LEVER_M_T2 = "
if marker in text:
    text = re.sub(r"LEVER_M_T2 = [\d.]+", f"LEVER_M_T2 = {mm / 1000.0}", text)
else:
    text = text.replace(
        "# The lever magnitude carried by S2 and S3 [m].\nLEVER_M = 0.060",
        "# The lever magnitude carried by S2 and S3 [m]. The t2 axis takes its\n"
        "# own value: the face is 20 mm half-width across it against 60 mm\n"
        "# across t1, so the same lever turns the tool much further there and a\n"
        "# 60 mm setting overshot zero by 15 deg.\n"
        f"LEVER_M = 0.060\nLEVER_M_T2 = {mm / 1000.0}")
open(path, "w").write(text)
print(f"generator now uses {mm} mm about t2")
EOF

python3 "$GEN"
exec "$HERE/run_campaign.sh" all
