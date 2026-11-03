"""Per-test DDS isolation for the launch tests in this package.

`colcon test` runs these launch tests concurrently. They all bring up a
`/perception_container` and all probe the same topic names, so on a shared
ROS_DOMAIN_ID they see each other's traffic: the symptom is inflated counts
(a 278-cloud replay probe reporting 597 clouds) and T9 "misses" that are
really another test's frames arriving without that test's TF. Nothing about
the product is wrong; the harness was talking to itself.

Each test module calls `isolate(n)` at import time, BEFORE rclpy.init and
before any launch action is constructed, so both the test process and every
process it launches inherit the domain. Ids are hand-assigned and unique;
keep them under 101 (the range that needs no kernel tuning) and never use 0 —
that is the simulator's domain (§12.2) and `test_bringup_sim` deliberately
lives there to talk to a live sim.
"""
import os

# Registry — one line per launch test, so collisions are visible at a glance.
WALL_ACCURACY = 41
PROJECTION_REPLAY = 42
DETECTION_STATIC = 43
TRACKING_05 = 44
TRACKING_08 = 45
MARKER_RELAY = 46
HW_SOURCE_CONTRACT = 47
DLIO_WIRING = 48
T8_DETERMINISM = 49


def isolate(domain_id):
    """Pin this process (and its children) to a private DDS domain.

    Unconditional by design: the shell that runs `colcon test` almost always
    exports ROS_DOMAIN_ID=0 (the workspace's documented default), so an
    "only if unset" rule would silently never fire. Set
    PERCEPTION_TEST_DOMAIN=<id> to force a specific domain — e.g. 0 to point a
    single test at a live simulator by hand.
    """
    forced = os.environ.get('PERCEPTION_TEST_DOMAIN')
    os.environ['ROS_DOMAIN_ID'] = forced if forced else str(domain_id)
    return os.environ['ROS_DOMAIN_ID']
