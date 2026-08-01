"""T5 dynamic-tracking gate at 0.5 m/s (§16.2, Phase 3) — see
tracking_dynamic_common.py.

Standalone:
  launch_test src/g1_perception/g1_perception_bringup/test/test_tracking_dynamic_05.launch_test.py
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# NOTE: the base class must NOT be bound in this module's namespace or
# launch_testing collects and runs it with its class attributes unset.
import tracking_dynamic_common as tdc   # noqa: E402

BAG = os.path.join(tdc.WS, 'test_fixtures', 's2_cross_05')
SKIP_REASON = (None if os.path.isdir(BAG)
               else f'fixture bag missing: {BAG} (gitignored; see README)')


@pytest.mark.launch_test
def generate_test_description():
    return tdc.make_description(BAG, SKIP_REASON)


class TestTrackingDynamic05(tdc.TrackingDynamicBase):
    SKIP_REASON = SKIP_REASON
    SPEED = 0.5
    COLLECT_S = 30.0   # 21.6 s bag + startup margin
