# ROS 2 Communication Setup (Fast DDS)

How to make two computers see each other's ROS 2 topics: the robot's onboard PC
publishing, and your laptop watching.

The link uses **Fast DDS** (`rmw_fastrtps_cpp`), which is the default middleware
of both ROS 2 Foxy and Humble. Nothing extra has to be installed on either
machine, and there is no XML to write by hand.

| | Onboard PC | Laptop |
|---|---|---|
| ROS 2 | Foxy (Ubuntu 20.04) | Humble (Ubuntu 22.04) |
| Role | publishes odometry and obstacles | subscribes and displays |

The laptop only subscribes. Closing it, losing Wi-Fi, or killing the viewer has
no effect on the robot.

---

## 1. What must match

Three variables. Get these wrong and the topic list can still look plausible
while no data ever arrives.

| Variable | Value | Rule |
|---|---|---|
| `ROS_DOMAIN_ID` | any `0`–`101` | **identical on both machines**. Do not use `0` — it is everyone's default, so anything else on the network joins you |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | **identical on both machines**. Different DDS vendors never interoperate |
| `ROS_LOCALHOST_ONLY` | `0` | `1` means traffic can never leave the machine. This is the single most common cause of "the topic list looks right but I see nothing" |

---

## 2. Write `~/.g1_net_env` on each machine

Do this **separately on each computer** — one line differs between them.

First confirm where the repository actually is on *this* machine:

```bash
cd ~/unitree_rl_mjlab/ros2 && pwd
# if that fails, find it:
find ~ -maxdepth 5 -name deps.repos -path "*/ros2/*" 2>/dev/null
```

Then write the file, putting the path you just saw into `G1_WS`:

```bash
cat > ~/.g1_net_env <<'EOF'
export ROS_DOMAIN_ID=7                        # SAME on both computers
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp    # SAME on both computers
export ROS_LOCALHOST_ONLY=0
unset CYCLONEDDS_URI
export G1_WS=$HOME/unitree_rl_mjlab/ros2      # this machine's own path
#export G1_PEER_IP=192.168.0.xxx              # the OTHER computer; only for §4
EOF

echo '[ -f ~/.g1_net_env ] && . ~/.g1_net_env' >> ~/.bashrc
```

Everything in that file is a real ROS 2 variable, so sourcing it is by itself
enough to put a shell on the link. Adding the `.bashrc` line means you cannot
forget it in a new terminal.

Open a **new terminal** and confirm:

```bash
printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY | tr '\n' ' '; echo
ls "$G1_WS/deps.repos"
```

Expected: `7 rmw_fastrtps_cpp 0`, and the path to `deps.repos`.

If `ls` fails, `G1_WS` is wrong. Fix it now — otherwise every later
`cd "$G1_WS"` runs in the wrong directory and it looks like "I built it but the
packages are gone".

> `G1_WS` is **not** a ROS variable. It exists only so copy-paste blocks can say
> `cd "$G1_WS"` on both machines, and its value **differs per machine** because
> the two computers cloned the repository to different places. Do not copy the
> other machine's line.

---

## 3. Check the link, in order

Two scripts do the checking. Both are read-only — they start no part of the
robot stack and publish nothing the robot could act on.

```
ros2/src/g1_perception/g1_perception_bringup/scripts/net_env.sh
ros2/src/g1_perception/g1_perception_bringup/scripts/g1_link_check.sh
```

### 3.1 Audit each machine's environment

Run on **both** machines:

```bash
"$G1_WS"/src/g1_perception/g1_perception_bringup/scripts/g1_link_check.sh
```

```
    ok   ROS_DOMAIN_ID=7   <- compare with the other machine
    ok   RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    ok   ROS_LOCALHOST_ONLY=0
    ok   no Fast DDS profile file (multicast discovery, the default)
LINK ENV OK.
```

Read the `ROS_DOMAIN_ID` on both screens and confirm with your own eyes that it
is the same number.

### 3.2 Does this network pass multicast?

On the **laptop**, leave this waiting:

```bash
"$G1_WS"/src/g1_perception/g1_perception_bringup/scripts/g1_link_check.sh recv
```

Then on the **onboard PC**:

```bash
"$G1_WS"/src/g1_perception/g1_perception_bringup/scripts/g1_link_check.sh send
```

| Laptop shows | Verdict | Next |
|---|---|---|
| `Received from 192.168.x.x: 'Hello World!'` | multicast works | nothing more to do — skip §4 |
| nothing | multicast is blocked | go to §4 |

### 3.3 Do the topics actually arrive?

With the stack running on the onboard PC, on the **laptop**:

```bash
ros2 daemon stop        # required whenever you changed the environment
"$G1_WS"/src/g1_perception/g1_perception_bringup/scripts/g1_link_check.sh topics
```

```
    ok   /odom is on the graph
    ok   /obstacles_safe is on the graph
```

Then check that data, not just names, is flowing:

```bash
ros2 topic hz /odom --no-daemon
ros2 topic echo /obstacles_safe --once
```

---

## 4. When multicast is blocked (peers mode)

Common on lab and campus Wi-Fi. Instead of discovering each other by multicast,
each machine is told the other's address directly.

On **both** machines, uncomment `G1_PEER_IP` in `~/.g1_net_env` and set it to
the **other** computer's IP. The two values are opposites:

| | `G1_PEER_IP` holds |
|---|---|
| onboard PC | the **laptop's** IP |
| laptop | the **onboard PC's** IP |

Find your own address with `ip -br addr`. Then, in each terminal:

```bash
source ~/.g1_net_env
source "$G1_WS"/src/g1_perception/g1_perception_bringup/scripts/net_env.sh peers
```

```
    ok   FASTRTPS_DEFAULT_PROFILES_FILE=/home/<user>/.g1_fastdds.xml
         unicast discovery peer: 192.168.x.x  (the OTHER computer must list THIS one)
    PASS - this shell is on the link.
```

The script writes the Fast DDS profile to `~/.g1_fastdds.xml` itself, so there
is no XML to maintain and no install-prefix path to resolve.

Peers mode must be enabled on **both** sides, and it applies **only to the
terminal you ran it in**. To make it permanent, add this below the
`.g1_net_env` line in `~/.bashrc`:

```bash
source "$G1_WS"/src/g1_perception/g1_perception_bringup/scripts/net_env.sh peers >/dev/null
```

---

## 5. Special case: Foxy and Humble on one machine

If you rehearse with a Foxy container and Humble on the same host, **they will
not find each other with default settings**. Humble's Fast DDS 2.6 advertises a
shared-memory locator that Foxy's Fast DDS 2.0 cannot use. On both sides:

```bash
source "$G1_WS"/src/g1_perception/g1_perception_bringup/scripts/net_env.sh udp-only
```

Two separate machines never use shared memory, so **do not enable this in the
field** — forcing UDP makes large topics travel over the network stack instead
of memory and costs CPU on the robot.

---

## 6. Troubleshooting

"The laptop sees no topics at all" — work down this list. It is ordered by how
often each one turns out to be the answer.

| # | Check | Command | If wrong |
|---|---|---|---|
| 1 | stale daemon cache | `ros2 daemon stop`, retry (or add `--no-daemon`) | — |
| 2 | same domain? | `echo $ROS_DOMAIN_ID` on both | fix `~/.g1_net_env`, then open a **new terminal** |
| 3 | same middleware? | `echo $RMW_IMPLEMENTATION` on both | both must be `rmw_fastrtps_cpp` |
| 4 | localhost jail | `echo $ROS_LOCALHOST_ONLY` on both | must be `0` |
| 5 | do the IPs even reach? | `ping <other IP>` | a network problem, not a DDS one — fix this first |
| 6 | multicast | §3.2 | switch to peers mode (§4) |
| 7 | firewall | `sudo ufw status` | open UDP **7400–7500**, or `sudo ufw disable` |
| 8 | is the robot publishing at all? | `ros2 topic hz /odom` **on the onboard PC** | not a link problem — the stack is down |

### `ros2 topic list` works but `hz` shows nothing

The path is open and the publisher is the problem. Measure the same topic on
the onboard PC.

### `failed to load rmw implementation`

`RMW_IMPLEMENTATION` names a middleware that is not installed. Fast DDS ships
with both distros, so this is a typo or a leftover CycloneDDS setting:

```bash
grep -rn "RMW_IMPLEMENTATION\|CYCLONEDDS" ~/.bashrc ~/.profile ~/.g1_net_env
```

### Changing the environment appears to do nothing

`ros2 daemon` caches the graph. After any change to domain, middleware or
profile, run `ros2 daemon stop` — or pass `--no-daemon` to the `ros2` command.
 