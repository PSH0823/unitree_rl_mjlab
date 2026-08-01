"""T5 dynamic-tracking gate at 0.8 m/s (§16.2, Phase 3) — see
tracking_dynamic_common.py.

Standalone:
  launch_test src/g1_perception/g1_perception_bringup/test/test_tracking_dynamic_08.launch_test.py
"""

# Concurrent launch tests share topic names; give this one a private DDS
# domain before anything else touches ROS (see isolate_domain.py).
# launch_test loads this file by path without putting its directory on
# sys.path, so the sibling import needs the insert first.
import os as _os, sys as _sys  # noqa: E402
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))  # noqa: E402
import isolate_domain  # noqa: E402
isolate_domain.isolate(isolate_domain.TRACKING_08)  # noqa: E402
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# NOTE: the base class must NOT be bound in this module's namespace or
# launch_testing collects and runs it with its class attributes unset.
import tracking_dynamic_common as tdc   # noqa: E402

BAG = os.path.join(tdc.WS, 'test_fixtures', 's2_cross_08')
SKIP_REASON = (None if os.path.isdir(BAG)
               else f'fixture bag missing: {BAG} (gitignored; see README)')


@pytest.mark.launch_test
def generate_test_description():
    return tdc.make_description(BAG, SKIP_REASON)


class TestTrackingDynamic08(tdc.TrackingDynamicBase):
    SKIP_REASON = SKIP_REASON
    SPEED = 0.8
    COLLECT_S = 23.0   # 14.5 s bag + startup margin
