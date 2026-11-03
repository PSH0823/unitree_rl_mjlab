#!/usr/bin/env python3
"""Per-process CPU over a window, as a fraction of one core (§17.4 budget).

  proc_cpu.py <seconds> <name-substring> [more names...]

Reads utime+stime from /proc/<pid>/stat directly rather than shelling out to
top or ps: those report an average since process start (ps) or over their own
sampling interval (top), neither of which is the window you asked about. Sums
over every matching pid so a multi-process match (the marker relays, RViz's
helpers) is not silently attributed to whichever one was listed first.
"""
import os
import sys
import time

HZ = os.sysconf('SC_CLK_TCK')


def pids_matching(name):
    """Pids whose cmdline contains `name`, excluding this process.

    Excluding self is not cosmetic: our own argv carries every name we were
    asked to search for, so without this each target matches the measuring
    process too and the report gains a phantom pid. Same self-match class as
    the `pkill -f component_container` trap (runbook §3.4).
    """
    me = os.getpid()
    out = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit() or int(entry) == me:
            continue
        try:
            with open(f'/proc/{entry}/cmdline', 'rb') as f:
                cmdline = f.read().replace(b'\0', b' ').decode('utf-8', 'replace')
        except OSError:
            continue
        if name in cmdline:
            out.append(int(entry))
    return out


def ticks(pid):
    try:
        with open(f'/proc/{pid}/stat') as f:
            # comm may contain spaces; everything after the final ')' is safe.
            fields = f.read().rsplit(')', 1)[1].split()
        return int(fields[11]) + int(fields[12])   # utime + stime
    except (OSError, IndexError, ValueError):
        return None


def main():
    secs = float(sys.argv[1])
    names = sys.argv[2:]
    first = {}
    for name in names:
        first[name] = {pid: ticks(pid) for pid in pids_matching(name)}
    t0 = time.time()
    time.sleep(secs)
    span = time.time() - t0

    print(f'CPU over {span:.1f}s, % of one core (utime+stime from /proc)')
    for name in names:
        used = 0
        seen = 0
        for pid, start in first[name].items():
            end = ticks(pid)
            if start is None or end is None:
                continue
            used += end - start
            seen += 1
        if seen == 0:
            print(f'  {name:24s} NOT RUNNING')
            continue
        pct = 100.0 * (used / HZ) / span
        print(f'  {name:24s} {pct:6.2f}%   ({seen} pid(s))')
    return 0


if __name__ == '__main__':
    sys.exit(main())
