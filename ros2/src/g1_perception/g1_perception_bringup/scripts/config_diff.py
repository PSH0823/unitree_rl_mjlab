#!/usr/bin/python3
"""Source-vs-installed config/launch comparison (Phase 5C).

Launch files, parameter YAMLs, the RViz layout and the Mid-360 JSON are
INSTALLED ARTEFACTS: `install(DIRECTORY launch config rviz ...)` copies them
into `install/share/g1_perception_bringup/`, and that copy — not the file you
edited — is what `ros2 launch` reads. Editing a YAML and launching without
`colcon build` is the single most expensive way to waste robot time, because
nothing anywhere reports it: the stack comes up, with the old numbers.

This prints, for every tracked file, whether the installed copy is IDENTICAL,
DIFFERENT or MISSING, plus both checksums. Also usable as an evidence
artefact: `--json` writes the checksums that belong next to a hardware bag.

  ros2 run g1_perception_bringup config_diff.py
  ros2 run g1_perception_bringup config_diff.py --json evidence/.../configs.json

Exit 0 = every installed copy matches source. Exit 1 = drift (or missing
install prefix). Exit 3 = source tree not found (running from an install-only
machine, where there is nothing to compare and that is fine).
"""
import argparse
import hashlib
import json
import os
import sys

SUBDIRS = ('launch', 'config', 'rviz')


def md5(path):
    h = hashlib.md5()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def find_source_dir(explicit=None):
    if explicit:
        return explicit
    # scripts/ lives directly under the package source root.
    here = os.path.dirname(os.path.abspath(__file__))
    cand = os.path.abspath(os.path.join(here, '..'))
    if os.path.isdir(os.path.join(cand, 'launch')):
        return cand
    return None


def find_install_dir(explicit=None):
    if explicit:
        return explicit
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('g1_perception_bringup')
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source')
    ap.add_argument('--install')
    ap.add_argument('--json')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()

    src = find_source_dir(args.source)
    inst = find_install_dir(args.install)
    if src is None:
        print('config_diff: no source tree next to this script — nothing to '
              'compare (install-only machine).')
        sys.exit(3)
    if inst is None or not os.path.isdir(inst):
        print('config_diff: FAIL — package share directory not found. The '
              'workspace is not built or not sourced; `ros2 launch` will not '
              'find these files either.')
        sys.exit(1)

    rows, drift = [], 0
    for sub in SUBDIRS:
        sdir = os.path.join(src, sub)
        if not os.path.isdir(sdir):
            continue
        for name in sorted(os.listdir(sdir)):
            sp = os.path.join(sdir, name)
            if not os.path.isfile(sp):
                continue
            ip = os.path.join(inst, sub, name)
            s = md5(sp)
            if not os.path.exists(ip):
                rows.append((f'{sub}/{name}', 'MISSING', s, ''))
                drift += 1
                continue
            i = md5(ip)
            state = 'identical' if s == i else 'DIFFERENT'
            if s != i:
                drift += 1
            rows.append((f'{sub}/{name}', state, s, i))

    # Files only present in the install prefix: a rename or deletion in source
    # leaves the old artefact behind, and `ros2 launch <old name>` still works.
    stale = []
    for sub in SUBDIRS:
        idir = os.path.join(inst, sub)
        if not os.path.isdir(idir):
            continue
        for name in sorted(os.listdir(idir)):
            if not os.path.isfile(os.path.join(idir, name)):
                continue
            if not os.path.exists(os.path.join(src, sub, name)):
                stale.append(f'{sub}/{name}')

    if not args.quiet:
        print(f'source : {src}')
        print(f'install: {inst}\n')
        w = max((len(r[0]) for r in rows), default=10)
        for name, state, s, i in rows:
            print(f'  {name:<{w}}  {state:<9}  {s[:12]}  {i[:12]}')
        if stale:
            print('\nSTALE INSTALLED ARTEFACTS (present in install, absent in '
                  'source — a rebuild does NOT remove these):')
            for n in stale:
                print('  -', n)

    if args.json:
        with open(args.json, 'w') as f:
            json.dump({'source_dir': src, 'install_dir': inst,
                       'files': [{'path': n, 'state': st,
                                  'source_md5': s, 'installed_md5': i}
                                 for n, st, s, i in rows],
                       'stale_installed': stale}, f, indent=2)
        print(f'\nJSON: {args.json}')

    if drift or stale:
        print(f'\nconfig_diff: FAIL — {drift} file(s) differ or are missing, '
              f'{len(stale)} stale artefact(s).\n'
              '  cd ros2 && colcon build --merge-install '
              '--packages-select g1_perception_bringup && '
              'source install/setup.bash\n'
              '  (stale artefacts survive a rebuild: remove '
              'build/ and install/share/g1_perception_bringup to clear them)')
        sys.exit(1)
    print('\nconfig_diff: PASS — installed copies match source')
    sys.exit(0)


if __name__ == '__main__':
    main()
