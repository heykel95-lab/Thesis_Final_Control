#!/usr/bin/env python3
"""Drive one trial of run.sh without an operator at the keyboard.

usage: auto_drive.py <run_id> <repeat_index>

The controller asks for input at three points. This answers each one the moment
its prompt appears, with exactly what a hand at the keyboard would type, so the
protocol is the one an operator would follow, gates included. What changes is
only that the gate waits become instant and identical every trial rather than
however long the operator took.

That matters more than it used to. The set-up push ramp runs on the live phase
clock, so a wait at the pre-grinding gate keeps building preload until the
configured final penetration is reached. An instant, identical answer removes
the operator's reaction time from the recorded load.

Reading is by chunk, not by line: the startup menu prompt ends in ": " with no
newline, so a line-based reader blocks on it forever.
"""

import os
import signal
import subprocess
import sys
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
RUN_SH = os.path.join(HERE, "..", "run.sh")


def replies_for(run_id):
    """Prompt fragment to answer, and what to send.

    The startup key comes from the setup rather than being assumed: a contact
    trial runs the sequence with s, and a pose-hold trial holds with h. A hold
    run ends on its own experiment duration, so the gate answers are simply
    never matched.
    """
    mode = b"s"
    path = os.path.join(HERE, "..", "setups", run_id, "startup_mode.txt")
    if os.path.isfile(path):
        with open(path) as f:
            first = f.read().strip()
        if first in ("s", "h"):
            mode = first.encode()

    # A plain hold asks for its nullspace mode after reaching q_init. Enter
    # would keep the overlay's value, but answering with the digit puts the
    # mode the trial ran in its own transcript instead of leaving it implied.
    nullspace = b""
    overlay = os.path.join(HERE, "..", "setups", run_id, "overlay.txt")
    if os.path.isfile(overlay):
        with open(overlay) as f:
            for line in f:
                key, _, value = line.partition("=")
                if key.strip() == "nullspace_mode":
                    value = value.split("#")[0].strip()
                    if value in ("0", "1", "2", "3"):
                        nullspace = value.encode()

    return [
        (b"Selection [s/h/t/g/i/o/c/r/f/b/e]: ", mode + b"\n"),
        # The "Enter = N" tail of this prompt changes with the configured mode,
        # so match the stable part.
        (b"Selection [0/1/2/3", nullspace + b"\n"),
        (b"[GATE] Reached", b"\n"),          # start the set-up press
        (b"[GATE] Set up finished", b"e\n"),  # result has printed; stop
    ]


# A trial is about a minute of robot motion plus archiving; well past that and
# something is waiting on input this script does not know how to answer.
TRIAL_TIMEOUT_S = 600


def main():
    if len(sys.argv) < 3:
        sys.exit("usage: auto_drive.py <run_id> <repeat_index>")
    run_id, repeat = sys.argv[1], sys.argv[2]

    replies = replies_for(run_id)

    # Its own process group, so a timeout takes the controller down with
    # run.sh. Killing run.sh alone leaves the controller holding the robot.
    proc = subprocess.Popen(
        [RUN_SH, run_id, repeat],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    def kill_group():
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            proc.kill()

    timed_out = threading.Event()
    timer = threading.Timer(TRIAL_TIMEOUT_S, lambda: (timed_out.set(),
                                                      kill_group()))
    timer.daemon = True
    timer.start()

    out = sys.stdout.buffer
    buf = b""
    try:
        while True:
            chunk = os.read(proc.stdout.fileno(), 4096)
            if not chunk:
                break
            out.write(chunk)
            out.flush()
            buf += chunk
            for pattern, reply in replies:
                at = buf.find(pattern)
                if at < 0:
                    continue
                try:
                    proc.stdin.write(reply)
                    proc.stdin.flush()
                except (BrokenPipeError, ValueError):
                    pass
                # Drop through the match so the same prompt cannot fire twice.
                buf = buf[at + len(pattern):]
                break
            if len(buf) > 8192:
                buf = buf[-4096:]
    finally:
        timer.cancel()
        try:
            proc.stdin.close()
        except (BrokenPipeError, ValueError):
            pass

    code = proc.wait()
    if timed_out.is_set():
        sys.exit(f"TIMEOUT after {TRIAL_TIMEOUT_S} s: {run_id} r{repeat}")
    sys.exit(code)


if __name__ == "__main__":
    main()
