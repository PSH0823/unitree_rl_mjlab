#!/usr/bin/python3
"""Phase 5C offline gates — everything about the hardware path that can be
asserted WITHOUT a robot, a LiDAR, or a running ROS graph.

Deliberately not a launch test and deliberately not hardware-dependent: it is
a plain CTest that runs under system python (R-11) on any machine, so the
guarantees below are checked on every build rather than on robot day.

  1  the shipped Mid-360 config is detected as placeholder (exit 2)
  2  a non-placeholder but non-local host IP is rejected (exit 1)
  3  source-vs-installed config drift is detected
  4  the perception-only launch closure contains no command publisher,
     no g1_ctrl / deploy / Unitree node, and no DPCBF actuation seam
  5  the TF probe looks transforms up at the MESSAGE STAMP, never latest
  6  the probe core detects stamp regression, wrong frame_id, wrong layout
  7  recording metadata reports its missing operator fields
  8  ground_seg:=patchwork is rejected with an explanation
  9  every hardware launch file constructs
 10  the diagnostics no-data rule is ERROR, not silence

Hardware-dependent checks live in doc/g1_first_perception_experiment.md and
are run by hand on the robot. Nothing here touches a device.

Exit 0 pass, 1 fail.
"""
import ast
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, '..'))
SCRIPTS = os.path.join(PKG, 'scripts')
LAUNCH = os.path.join(PKG, 'launch')
CONFIG = os.path.join(PKG, 'config')
PY = '/usr/bin/python3'

sys.path.insert(0, SCRIPTS)
import hw_probe_core as core            # noqa: E402

FAILURES = []
PASSED = []


def check(name, cond, detail=''):
    if cond:
        PASSED.append(name)
        print(f'  PASS  {name}')
    else:
        FAILURES.append(f'{name}: {detail}')
        print(f'  FAIL  {name}: {detail}')


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


# --- 1/2 network configuration gates --------------------------------------
def test_network_config():
    print('\n[1/2] Mid-360 network configuration')
    hw_check = os.path.join(PKG, 'test', 'hw_config_check.py')
    r = run([PY, hw_check, os.path.join(CONFIG, 'MID360_config.json'),
             os.path.join(CONFIG, 'livox_driver.yaml')])
    check('placeholder IPs are rejected (exit 2)', r.returncode == 2,
          f'exit {r.returncode}; output: {r.stdout[-300:]}')
    check('placeholder rejection says so in words',
          'PLACEHOLDER' in r.stdout, r.stdout[-200:])

    # A config that is NOT the upstream placeholder but still names a host IP
    # this machine does not own: the SDK binds those addresses, so this is the
    # "driver starts, prints nothing unusual, /livox/lidar stays empty" case.
    with open(os.path.join(CONFIG, 'MID360_config.json')) as f:
        cfg = json.load(f)
    host = cfg['MID360']['host_net_info']
    for k in list(host):
        if k.endswith('_ip') and host[k]:
            host[k] = '10.253.253.253'
    cfg['lidar_configs'][0]['ip'] = '10.253.253.254'
    with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as f:
        json.dump(cfg, f)
        bogus = f.name
    r = run([PY, hw_check, bogus, os.path.join(CONFIG, 'livox_driver.yaml')])
    check('unassigned host IP is rejected (exit 1)', r.returncode == 1,
          f'exit {r.returncode}; output: {r.stdout[-300:]}')
    check('rejection names the bind failure',
          'not assigned to any local interface' in r.stdout, r.stdout[-200:])
    os.unlink(bogus)


# --- 3 source vs installed -------------------------------------------------
def test_config_diff():
    print('\n[3] source-vs-installed drift detection')
    diff = os.path.join(SCRIPTS, 'config_diff.py')
    tmp = tempfile.mkdtemp()
    src = os.path.join(tmp, 'src')
    inst = os.path.join(tmp, 'install')
    for d in (src, inst):
        os.makedirs(os.path.join(d, 'config'))
        os.makedirs(os.path.join(d, 'launch'))
    shutil.copy(os.path.join(CONFIG, 'dlio.yaml'),
                os.path.join(src, 'config', 'dlio.yaml'))
    shutil.copy(os.path.join(CONFIG, 'dlio.yaml'),
                os.path.join(inst, 'config', 'dlio.yaml'))
    r = run([PY, diff, '--source', src, '--install', inst, '--quiet'])
    check('identical trees pass', r.returncode == 0, r.stdout[-200:])

    with open(os.path.join(inst, 'config', 'dlio.yaml'), 'a') as f:
        f.write('\n# stale installed copy\n')
    r = run([PY, diff, '--source', src, '--install', inst, '--quiet'])
    check('a differing installed copy fails', r.returncode == 1,
          f'exit {r.returncode}')
    check('the failure names the rebuild command',
          'colcon build' in r.stdout, r.stdout[-200:])

    # A file only in the install prefix survives every rebuild and stays
    # launchable by its old name.
    shutil.copy(os.path.join(CONFIG, 'dlio.yaml'),
                os.path.join(inst, 'config', 'deleted_from_source.yaml'))
    r = run([PY, diff, '--source', src, '--install', inst])
    check('stale installed artefacts are reported',
          'STALE INSTALLED ARTEFACTS' in r.stdout, r.stdout[-300:])
    shutil.rmtree(tmp)


# --- 4 perception-only launch isolation -----------------------------------
# Nothing in the perception-only closure may reference any of these. The list
# is written as regexes over the launch SOURCE so that a future edit that
# adds a controller node trips the gate at build time.
FORBIDDEN_PACKAGES = (
    'g1_ctrl', 'unitree_mujoco', 'unitree_sdk2', 'unitree_ros2',
    'deploy', 'dpcbf_ros_adapter',
)
FORBIDDEN_TOPICS = (
    '/cmd_vel', 'rt/lowcmd', 'rt/wirelesscontroller', 'rt/api', '/lowcmd',
    '/dpcbf/command', 'sportmodestate', 'arm_sdk',
)
HW_CLOSURE = [
    'g1_perception_hardware_only.launch.py',
    'source_hw.launch.py',
    'description.launch.py',
    'perception.launch.py',
    'record_hw.launch.py',
]


def _launch_nodes(path):
    """Package/executable pairs a launch file constructs, via AST — a text
    grep would also match the prose in the docstrings, which is exactly where
    the forbidden names legitimately appear."""
    tree = ast.parse(open(path).read())
    out = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        fn = node.func
        name = getattr(fn, 'id', None) or getattr(fn, 'attr', None)
        if name not in ('Node', 'ComposableNode', 'LifecycleNode',
                        'ExecuteProcess'):
            continue
        kw = {k.arg: k.value for k in node.keywords}
        pkg = _const(kw.get('package'))
        exe = _const(kw.get('executable')) or _const(kw.get('plugin'))
        cmd = kw.get('cmd')
        out.append({'kind': name, 'package': pkg, 'executable': exe,
                    'cmd': ast.dump(cmd) if cmd is not None else ''})
    return out


def _const(node):
    if isinstance(node, ast.Constant):
        return node.value
    return None


def code_only(path):
    """Source with comments and docstrings removed.

    Necessary because these files DISCUSS the things they must not do — the
    hardware launch explains why it does not include bringup.launch.py, the
    TF probe explains why a TimePointZero lookup would be worthless. A plain
    grep would fail the gate on its own documentation, which would teach the
    next person to delete the documentation.
    """
    if not hasattr(ast, 'unparse'):
        return _code_only_tokenize(path)
    tree = ast.parse(open(path).read())
    for node in ast.walk(tree):
        if not isinstance(node, (ast.Module, ast.FunctionDef,
                                 ast.AsyncFunctionDef, ast.ClassDef)):
            continue
        body = node.body
        if (body and isinstance(body[0], ast.Expr)
                and isinstance(body[0].value, ast.Constant)
                and isinstance(body[0].value.value, str)):
            node.body = body[1:] or [ast.Pass()]
    return ast.unparse(ast.fix_missing_locations(tree))


def _code_only_tokenize(path):
    """code_only() for Python 3.8 — the G1's onboard computer runs Foxy.

    ast.unparse() is 3.9+. This drops the same two things by tokenising:
    COMMENT tokens, and STRING tokens standing alone as a statement (module,
    class and function docstrings, and any bare string expression). The result
    is one token per line rather than reconstructed source, which is enough
    because callers only substring-search it and every name and string literal
    survives inside a single token.
    """
    import tokenize
    keep, statement_start = [], True
    with open(path, 'rb') as fh:
        for tok in tokenize.tokenize(fh.readline):
            if tok.type == tokenize.COMMENT:
                continue
            if tok.type == tokenize.STRING and statement_start:
                continue                       # docstring / bare string
            if tok.type in (tokenize.NEWLINE, tokenize.NL, tokenize.INDENT,
                            tokenize.DEDENT, tokenize.ENCODING):
                statement_start = True
                continue
            keep.append(tok.string)
            statement_start = False
    return '\n'.join(keep)


def test_launch_isolation():
    print('\n[4] perception-only launch isolation (no actuation path)')
    all_nodes = []
    for name in HW_CLOSURE:
        p = os.path.join(LAUNCH, name)
        check(f'{name} exists', os.path.isfile(p), p)
        if os.path.isfile(p):
            all_nodes += [(name, n) for n in _launch_nodes(p)]

    for fname, n in all_nodes:
        pkg = (n['package'] or '') + ' ' + (n['executable'] or '')
        for bad in FORBIDDEN_PACKAGES:
            check(f'{fname}: no {bad} node', bad not in pkg,
                  f'{n}')
        for bad in FORBIDDEN_TOPICS:
            check(f'{fname}: no {bad} in a process command',
                  bad not in n['cmd'], f'{n}')

    # The hardware entry point must not offer an argument that could turn
    # actuation, ground segmentation or sim time on.
    top = open(os.path.join(LAUNCH, HW_CLOSURE[0])).read()
    args = set(re.findall(r"DeclareLaunchArgument\(\s*'([a-z_]+)'", top))
    check('hardware-only launch declares no `mode` argument',
          'mode' not in args, sorted(args))
    check('hardware-only launch declares no `ground_seg` argument',
          'ground_seg' not in args, sorted(args))
    check('hardware-only launch declares no `use_sim_time` argument',
          'use_sim_time' not in args, sorted(args))
    for required in ('use_rviz', 'record', 'voxel', 'lidar_config',
                     'dlio_config', 'rviz_config'):
        check(f'hardware-only launch exposes {required}', required in args,
              sorted(args))
    check('hardware-only launch does not include bringup.launch.py',
          'bringup.launch.py' not in code_only(
              os.path.join(LAUNCH, HW_CLOSURE[0])),
          'it would re-introduce mode/ground_seg')
    check('hardware-only launch pins use_sim_time false',
          "use_sim_time='false'" in top, 'no /clock exists on hardware')
    check('hardware-only launch disables the GT overlay',
          "overlay='off'" in top,
          'dpcbf_overlay is ground-truth-consuming and there is no GT here')


# --- 5 stamped TF lookup ---------------------------------------------------
def test_tf_probe_is_stamped():
    print('\n[5] TF probe uses the message stamp, not latest')
    src = open(os.path.join(SCRIPTS, 'hw_tf_probe.py')).read()
    check('probe builds a Time from the message header',
          'Time.from_msg(msg.header.stamp)' in src, 'not found')
    check('probe never looks up at TimePointZero',
          'TimePointZero' not in code_only(
              os.path.join(SCRIPTS, 'hw_tf_probe.py')),
          'a latest-lookup defeats the probe')
    tree = ast.parse(src)
    stamped = False
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and getattr(node.func, 'attr', '') == 'lookup_transform'):
            third = node.args[2] if len(node.args) > 2 else None
            # `stamp` is the local holding Time.from_msg(...)
            if isinstance(third, ast.Name) and third.id == 'stamp':
                stamped = True
            elif isinstance(third, ast.Call):
                stamped = False
                break
    check('lookup_transform is called with the stamp variable', stamped,
          'the third positional argument is not the message stamp')


# --- 6 probe core detections ----------------------------------------------
def test_probe_core():
    print('\n[6] probe core failure detection')
    s = core.StampSeries('/livox/lidar', expect_hz=10.0)
    for i in range(10):
        s.add(100.0 + 0.1 * i, 100.0 + 0.1 * i)
    s.add(100.5, 101.05)                       # a stamp that went BACKWARDS
    check('stamp regression is detected', len(s.regressions) == 1,
          str(s.regressions))
    check('stamp regression is reported as a problem',
          any('REGRESSION' in p for p in s.problems()), str(s.problems()))

    empty = core.StampSeries('/livox/imu', expect_hz=200.0)
    check('a silent topic is a problem',
          any('NO MESSAGES' in p for p in empty.problems()),
          str(empty.problems()))

    check('driver default frame_id is rejected',
          core.check_frame_id('frame_default') is not None, '')
    check('the example-launch frame_id is rejected',
          'livox_frame' in (core.check_frame_id('livox_frame') or ''), '')
    check('mid360_link is accepted',
          core.check_frame_id('mid360_link') is None, '')

    good = list(core.LIVOX_FIELDS)
    fatal, dev = core.check_cloud_layout(good, core.LIVOX_POINT_STEP)
    check('the pinned driver layout is clean', not fatal and not dev,
          f'{fatal} {dev}')
    fatal, dev = core.check_cloud_layout(
        [('x', 0, core.PF_FLOAT32), ('y', 4, core.PF_FLOAT32)], 8)
    check('a cloud with no z is fatal', any('no ' in f for f in fatal), str(fatal))
    fatal, dev = core.check_cloud_layout(
        [('x', 0, core.PF_FLOAT32), ('y', 4, core.PF_FLOAT32),
         ('z', 8, core.PF_FLOAT32)], 12)
    check('a 12-byte xyz cloud is usable but flagged as not the driver',
          not fatal and any('point_step' in d for d in dev), f'{fatal} {dev}')

    check('a tight, stable offset reads as host-clock',
          'host-clock' in core.classify_time_domain([0.004, 0.005, 0.006]), '')
    check('a large offset reads as a device clock',
          'sensor-clock' in core.classify_time_domain([37.2, 37.3, 37.25]), '')

    check('an implausible |accel| is flagged',
          core.imu_plausibility([0.02] * 10, [0.0] * 10), '')
    check('gravity at rest is accepted',
          not core.imu_plausibility([9.79, 9.81, 9.80], [0.001] * 3), '')

    tf = core.TfLookupStats('mid360_link:odom')
    for _ in range(10):
        tf.record_fail('Lookup would require extrapolation into the future')
    check('future extrapolation is called out',
          any('FUTURE' in p for p in tf.problems()), str(tf.problems()))


# --- 7 recording metadata --------------------------------------------------
def test_metadata_completeness():
    print('\n[7] recording metadata completeness')
    meta = os.path.join(SCRIPTS, 'hw_session_metadata.py')
    tmp = tempfile.mkdtemp()
    out = os.path.join(tmp, 'session.json')
    r = run([PY, meta, '--out', out], cwd=PKG)
    check('a bare record is reported INCOMPLETE', r.returncode == 1,
          f'exit {r.returncode}: {r.stdout[-200:]}')
    for field in ('hardware.g1_variant', 'hardware.lidar_serial',
                  'session.robot_state', 'session.scenario'):
        check(f'{field} is listed as missing', field in r.stdout,
              r.stdout[-400:])
    rec = json.load(open(out))
    check('actuation_enabled is recorded false',
          rec['actuation_enabled'] is False, str(rec.get('actuation_enabled')))
    check('command_publisher_active is recorded false',
          rec['session']['command_publisher_active'] is False, '')
    for key in ('lidar_rate_hz', 'imu_rate_hz', 'odom_rate_hz',
                'tf_lookup_success', 'cloud_to_safe_p95_ms'):
        check(f'results.{key} exists and starts null',
              key in rec['results'] and rec['results'][key] is None, '')
    check('config checksums are captured',
          any(rec['config']['checksums'].values()),
          'no YAML was found to checksum')
    check('the commit is captured',
          rec['repo']['commit'] not in ('', 'unknown'), rec['repo']['commit'])

    r = run([PY, meta, '--out', out, '--g1-variant', 'G1-EDU',
             '--lidar-serial', 'X', '--lidar-firmware', 'Y',
             '--mounted-extrinsic', 'measured', '--robot-state', 'Passive',
             '--scenario', 'flat floor', '--operator', 'me'], cwd=PKG)
    check('a fully filled record passes', r.returncode == 0,
          f'exit {r.returncode}: {r.stdout[-200:]}')
    shutil.rmtree(tmp)


# --- 8/9 launch construction + patchwork rejection -------------------------
def test_launch_construction():
    print('\n[8/9] launch construction and ground_seg rejection')
    try:
        import importlib.util
        from launch import LaunchContext            # noqa: F401
    except Exception as exc:
        print(f'  SKIP  launch python package unavailable ({exc})')
        return

    spec = importlib.util.spec_from_file_location(
        'bringup_launch', os.path.join(LAUNCH, 'bringup.launch.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ctx = LaunchContext()
    ctx.launch_configurations['ground_seg'] = 'patchwork'
    try:
        mod._reject_patchwork(ctx)
        check('ground_seg:=patchwork is rejected', False, 'it was accepted')
    except RuntimeError as exc:
        check('ground_seg:=patchwork is rejected', True, '')
        check('the rejection explains that it is not implemented',
              'NOT IMPLEMENTED' in str(exc), str(exc))
        check('the rejection names what does reject the floor',
              'min_height' in str(exc), str(exc))
    ctx.launch_configurations['ground_seg'] = 'off'
    check('ground_seg:=off is accepted', mod._reject_patchwork(ctx) == [], '')

    # Constructing the hardware launches needs the install prefix (they call
    # get_package_share_directory at module scope). Skip cleanly without one.
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('g1_perception_bringup')
    except Exception:
        print('  SKIP  workspace not sourced; launch construction not exercised')
        return
    for name in HW_CLOSURE:
        spec = importlib.util.spec_from_file_location(
            name.replace('.', '_'), os.path.join(LAUNCH, name))
        m = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(m)
            m.generate_launch_description()
            check(f'{name} constructs', True, '')
        except Exception as exc:
            check(f'{name} constructs', False, f'{type(exc).__name__}: {exc}')


# --- 10 diagnostics no-data rule ------------------------------------------
def test_diagnostics_rules():
    print('\n[10] diagnostics no-data rule')
    lvl, msg = core.rate_level(0, 0.0, 10.0, None, 0.3)
    check('no data is ERROR', lvl == core.LEVEL_ERROR, f'{lvl} {msg}')
    lvl, _ = core.rate_level(50, 4.0, 10.0, 0.05, 0.3)
    check('half rate is ERROR', lvl == core.LEVEL_ERROR, str(lvl))
    lvl, _ = core.rate_level(50, 7.5, 10.0, 0.05, 0.3)
    check('slightly low rate is WARN', lvl == core.LEVEL_WARN, str(lvl))
    lvl, _ = core.rate_level(50, 10.0, 10.0, 2.0, 0.3)
    check('healthy rate with stale stamps is WARN',
          lvl == core.LEVEL_WARN, str(lvl))
    lvl, _ = core.rate_level(50, 10.0, 10.0, 0.05, 0.3)
    check('healthy is OK', lvl == core.LEVEL_OK, str(lvl))

    src = open(os.path.join(SCRIPTS, 'hw_diagnostics.py')).read()
    check('the diagnostics node publishes only /diagnostics',
          src.count('create_publisher') == 1 and "'/diagnostics'" in src,
          'it has more than one publisher')
    check('the diagnostics node uses the shared rule',
          'core.rate_level(' in src, 'the rule was re-implemented inline')


def main():
    print('Phase 5C offline hardware gates')
    test_network_config()
    test_config_diff()
    test_launch_isolation()
    test_tf_probe_is_stamped()
    test_probe_core()
    test_metadata_completeness()
    test_launch_construction()
    test_diagnostics_rules()

    print(f'\n{len(PASSED)} passed, {len(FAILURES)} failed')
    if FAILURES:
        print('\nFAILURES:')
        for f in FAILURES:
            print('  -', f)
        sys.exit(1)
    print('\nAll offline hardware gates pass. NOTE: these are the checks a '
          'machine with no robot can make.\nEverything about the real sensor, '
          'the real mount, real odometry and real timing is\nunverified until '
          'doc/g1_first_perception_experiment.md is executed.')
    sys.exit(0)


if __name__ == '__main__':
    main()
