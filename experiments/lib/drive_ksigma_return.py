#!/usr/bin/env python3
"""Drive the mode-2 trial that switches the conditioning on after the push.

usage: drive_ksigma_return.py <run_id> <repeat_index>

Same job as auto_drive.py for the startup prompts, plus one timed command the
plain driver has no way to express: the run begins at k_sigma = 0, so the
disturbance swings the elbow freely, and the gain is raised to its working
value only once the push has let go. That split is the whole point of the
trial. Holding the elbow and returning it look identical in a start-versus-
final reading, so the two have to happen in separate stretches of one run.

The trigger is the controller's own release cue rather than a wall clock, so
the switch stays tied to the disturbance even if the timeline is retimed. The
delay after it covers the release ramp: raising the gain while the push is
still fading would have the conditioning fighting the tail of the force, and
the recovery would start from a force that is not yet zero.
"""

import os
import signal
import subprocess
import sys
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
RUN_SH = os.path.join(HERE, "..", "run.sh")

# Raised gain, and how long after the release cue to send it [s]. The release
# ramp is 1 s; the extra second leaves the arm coasting on nothing before the
# conditioning takes hold, which is what makes the return legible on camera.
K_SIGMA_AFTER_RELEASE = 2.0
SWITCH_DELAY_S = 2.0

RELEASE_CUE = b"AUTOMATIC RELEASE START"

TRIAL_TIMEOUT_S = 600


def main():
    if len(sys.argv) < 3:
        sys.exit("usage: drive_ksigma_return.py <run_id> <repeat_index>")
    run_id, repeat = sys.argv[1], sys.argv[2]

    replies = [
        (b"Selection [s/h/t/g/i/o/c/r/f/b/e]: ", b"h\n"),
        (b"Selection [0/1/2/3", b"2\n"),
    ]

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

    def send(text):
        try:
            proc.stdin.write(text)
            proc.stdin.flush()
        except (BrokenPipeError, ValueError):
            pass

    switch = None

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

            # The switch is armed once, on the first release cue. A timer
            # rather than a blocking sleep, so the transcript keeps streaming
            # while the arm coasts.
            if switch is None and RELEASE_CUE in buf:
                command = f"k {K_SIGMA_AFTER_RELEASE}\n".encode()
                switch = threading.Timer(
                    SWITCH_DELAY_S,
                    lambda: (send(command),
                             out.write(b"\n>>> DRIVER: k_sigma -> "
                                       + str(K_SIGMA_AFTER_RELEASE).encode()
                                       + b" Nm\n"),
                             out.flush()))
                switch.daemon = True
                switch.start()

            for pattern, reply in replies:
                at = buf.find(pattern)
                if at < 0:
                    continue
                send(reply)
                buf = buf[at + len(pattern):]
                break
            if len(buf) > 8192:
                buf = buf[-4096:]
    finally:
        timer.cancel()
        if switch is not None:
            switch.cancel()
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
