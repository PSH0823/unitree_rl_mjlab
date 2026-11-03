#!/bin/bash
# Real-time contention probe for the Navigation stack.
#
# Answers one question: when perception and the controller share the onboard
# Orin NX, does the 1 kHz FSM control thread get starved?  It only reads
# /proc, sysstat and topic rates.  It publishes nothing, changes no affinity
# or priority, and cannot move the robot.
#
#   cpu_probe.sh <session_dir> <run_label> [duration_s]
#
# Run it three times with nothing else changed between runs:
#
#   A   g1_ctrl only, in Navigation, no goal sent   -> uncontended baseline
#   B   perception only, g1_ctrl not running        -> perception's real cost
#   C   both at once                                -> the A/B delta
#
# C on its own attributes nothing: without A there is no baseline for the FSM
# thread's involuntary preemptions, and without B there is no way to tell the
# controller's own cost from the perception stack's.
#
#   cpu_probe.sh ~/nav_diagnosis A 60
#
# The robot must hang on the gantry, or at minimum stand in FixStand with a
# zero command.  It must not walk during any of the three runs.
set -uo pipefail

DIR="${1:?usage: cpu_probe.sh <session_dir> <run_label> [duration_s]}"
RUN="${2:?run label, e.g. A, B or C}"
DUR="${3:-60}"

OUT="$DIR/run_${RUN}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
echo "=== cpu_probe run ${RUN}, ${DUR}s -> ${OUT}"

# --------------------------------------------------------------------------
# Provenance.  Every number below is meaningless if the power mode dropped to
# 15 W (cores 4-7 offline) or if the junction temperature is already at the
# throttle point, so record both before anything else.
# --------------------------------------------------------------------------
{
  echo "date        : $(date -Is)"
  echo "host/arch   : $(hostname) $(uname -m)"
  echo "kernel      : $(uname -r)"
  echo "commit      : $(git -C "$(dirname "$0")" rev-parse HEAD 2>/dev/null || echo n/a)"
  echo "run label   : $RUN"
  echo "duration_s  : $DUR"
  echo "nvpmodel    : $(nvpmodel -q 2>/dev/null | head -1)"
  echo "cpu online  : $(cat /sys/devices/system/cpu/online)"
  echo "tj_start_mC : $(cat /sys/class/thermal/thermal_zone8/temp 2>/dev/null)"
  echo "rt_runtime  : $(cat /proc/sys/kernel/sched_rt_runtime_us)/$(cat /proc/sys/kernel/sched_rt_period_us)"
  echo "cmdline     : $(cat /proc/cmdline)"
  echo "ROS_DISTRO  : ${ROS_DISTRO:-unset}"
  echo "RMW         : ${RMW_IMPLEMENTATION:-unset}"
  echo "DOMAIN_ID   : ${ROS_DOMAIN_ID:-unset(0)}"
  for c in $(seq 0 7); do
    f=/sys/devices/system/cpu/cpu$c/cpufreq/scaling_cur_freq
    [ -r "$f" ] && echo "cpu${c}_kHz   : $(cat "$f")"
  done
} | tee "$OUT/provenance.txt"

PID=$(pgrep -x g1_ctrl | head -1 || true)
if [ -n "$PID" ]; then
  echo "g1_ctrl pid : $PID" | tee -a "$OUT/provenance.txt"
else
  echo "g1_ctrl pid : NOT RUNNING (expected for run B)" | tee -a "$OUT/provenance.txt"
fi

# --------------------------------------------------------------------------
# Thread placement, scheduling class and priority.  This is the direct answer
# to "is the 1 kHz thread on CPU 0", and the thread count is the tell for
# unbounded ONNX Runtime intra-op pools: the controller's own threads are
# roughly FSM + policy + ros executor + DDS + keyboard, so anything past ~15
# is ORT spinning.
# --------------------------------------------------------------------------
FSM_TID=""
if [ -n "$PID" ]; then
  ps -L -o tid,comm,psr,cls,rtprio,ni,pcpu,stat -p "$PID" > "$OUT/threads_before.txt"
  echo "thread count: $(( $(wc -l < "$OUT/threads_before.txt") - 1 ))" | tee -a "$OUT/provenance.txt"
  FSM_TID=$(awk '$2=="FSM"{print $1; exit}' "$OUT/threads_before.txt")
  if [ -n "$FSM_TID" ]; then
    { echo "FSM tid     : $FSM_TID"
      echo "FSM affinity: $(taskset -pc "$FSM_TID" 2>&1 | sed 's/.*: //')"
    } | tee -a "$OUT/provenance.txt"
  else
    echo "FSM tid     : not found (is the FSM thread running?)" | tee -a "$OUT/provenance.txt"
  fi
fi

ctxsw() {  # involuntary context switches of one thread, or empty
  awk '/nonvoluntary_ctxt_switches/{print $2}' \
      "/proc/$PID/task/$1/status" 2>/dev/null
}

# --------------------------------------------------------------------------
# Take the counter snapshots and the sampled series over the SAME window, so
# the softirq delta and the preemption delta describe one interval.
# --------------------------------------------------------------------------
cp /proc/softirqs   "$OUT/softirqs_before.txt"
cp /proc/interrupts "$OUT/interrupts_before.txt"
S1=""; [ -n "$FSM_TID" ] && S1=$(ctxsw "$FSM_TID")

mpstat -P ALL 1 "$DUR" > "$OUT/mpstat.txt" 2>&1 &
JOBS=$!
vmstat 1 "$DUR" > "$OUT/vmstat.txt" 2>&1 &
JOBS="$JOBS $!"
timeout "$DUR" tegrastats --interval 1000 > "$OUT/tegrastats.txt" 2>&1 &
JOBS="$JOBS $!"
[ -n "$PID" ] && { pidstat -t -p "$PID" 1 "$DUR" > "$OUT/pidstat_g1_ctrl.txt" 2>&1 & JOBS="$JOBS $!"; }
pidstat -u -C 'dlio|livox|component_cont' 1 "$DUR" > "$OUT/pidstat_perception.txt" 2>&1 &
JOBS="$JOBS $!"

# Topic health.  For markers and odom the mean rate is worthless: sleep_until
# catches up after an overrun, so a loop that misses deadlines still averages
# 10 Hz.  Only max tells you a tick was late, which is why --window is small
# enough that one late tick still moves the reported maximum.
if command -v ros2 >/dev/null 2>&1; then
  for t in /navigation/markers /odom /obstacles_safe; do
    name=$(echo "$t" | tr '/' '_')
    timeout "$DUR" ros2 topic hz "$t" --window 50 > "$OUT/hz${name}.txt" 2>&1 &
    JOBS="$JOBS $!"
  done
fi

# Sample thread placement mid-run: SetCpu() pins once at thread creation, so a
# thread that migrated later would only show up in a second sample.
sleep $(( DUR / 2 ))
[ -n "$PID" ] && ps -L -o tid,comm,psr,cls,rtprio,ni,pcpu,stat -p "$PID" \
    > "$OUT/threads_mid.txt" 2>/dev/null

wait $JOBS 2>/dev/null

S2=""; [ -n "$FSM_TID" ] && S2=$(ctxsw "$FSM_TID")
cp /proc/softirqs   "$OUT/softirqs_after.txt"
cp /proc/interrupts "$OUT/interrupts_after.txt"
[ -n "$PID" ] && ps -L -o tid,comm,psr,cls,rtprio,ni,pcpu,stat -p "$PID" \
    > "$OUT/threads_after.txt" 2>/dev/null

# --------------------------------------------------------------------------
# Summary.  Everything a decision depends on, in one file.
# --------------------------------------------------------------------------
{
  echo "=== run $RUN, ${DUR}s, $(date -Is)"
  echo
  echo "--- FSM 1 kHz thread"
  if [ -n "$S1" ] && [ -n "$S2" ]; then
    echo "involuntary preemptions : $(( S2 - S1 )) over ${DUR}s = $(( (S2 - S1) / DUR ))/s"
    echo "  run A should be near 0. A jump to hundreds/s in run C is the direct"
    echo "  evidence that CPU 0 contention is real."
  else
    echo "involuntary preemptions : n/a (g1_ctrl or its FSM thread not running)"
  fi
  [ -n "$FSM_TID" ] && grep -E "^\s*$FSM_TID\s" "$OUT/threads_before.txt"
  echo
  echo "--- NET_RX softirq delta per CPU (LiDAR receive lands here)"
  paste <(grep NET_RX "$OUT/softirqs_before.txt") \
        <(grep NET_RX "$OUT/softirqs_after.txt") 2>/dev/null | \
    awk '{ printf "NET_RX delta:"; n=(NF-2)/2;
           for (i=2; i<=n+1; i++) printf " cpu%d=%d", i-2, $(i+n+1)-$i; print "" }'
  echo
  echo "--- eth0 hard IRQ delta per CPU"
  paste <(grep -iE 'eth0' "$OUT/interrupts_before.txt" | head -1) \
        <(grep -iE 'eth0' "$OUT/interrupts_after.txt" | head -1) 2>/dev/null | \
    awk '{ printf "eth0 IRQ delta:"; for (i=2; i<=9; i++) printf " cpu%d=%d", i-2, $(i+10)-$i; print "" }'
  echo
  echo "--- per-CPU busy (mpstat average row)"
  grep -E 'Average' "$OUT/mpstat.txt" 2>/dev/null | head -10
  echo
  echo "--- zram swap activity (any nonzero si/so is a latency spike in the 1 kHz loop)"
  awk 'NR>2 && ($7>0 || $8>0) {print; found=1} END{if(!found) print "si/so all zero"}' \
      "$OUT/vmstat.txt" 2>/dev/null
  echo
  echo "--- thermal / clock at end"
  echo "tj_end_mC   : $(cat /sys/class/thermal/thermal_zone8/temp 2>/dev/null)"
  for c in $(seq 0 7); do
    f=/sys/devices/system/cpu/cpu$c/cpufreq/scaling_cur_freq
    [ -r "$f" ] && echo "cpu${c}_kHz   : $(cat "$f")"
  done
  echo "  MAXN has no power cap, so a sustained run can throttle below 1984 MHz."
  echo "  If it does, the core budget is wrong and the answer is less work, not"
  echo "  a different CPU assignment."
  echo
  echo "--- topic rates (read max, not average)"
  for f in "$OUT"/hz_*.txt; do
    [ -e "$f" ] || continue
    echo "$(basename "$f"): $(tail -2 "$f" | tr '\n' ' ')"
  done
} | tee "$OUT/summary.txt"

echo
echo "=== done: $OUT"
echo "Compare run C against runs A and B.  A alone cannot show contention and"
echo "C alone cannot attribute it."
