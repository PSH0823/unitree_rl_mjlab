#!/usr/bin/env bash
# Foxy build environment driver.
#
#   ./tools/foxy_docker.sh build-image     # build/refresh the focal+foxy image
#   ./tools/foxy_docker.sh shell           # interactive shell in the container
#   ./tools/foxy_docker.sh build [PKGS..]  # colcon build inside the container
#   ./tools/foxy_docker.sh test  [PKGS..]  # colcon test inside the container
#   ./tools/foxy_docker.sh exec  CMD...    # arbitrary command inside
#
# The repository is bind-mounted at the SAME absolute path it has on the host,
# so paths in compiler errors and CMake caches are directly clickable.
#
# The Foxy build is kept in build_foxy/ install_foxy/ log_foxy/ so it never
# collides with the host's Humble build/ install/ log/. Both can coexist.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${REPO_ROOT}/ros2"
IMAGE="g1-perception:foxy"

# Packages this workspace owns. External/ is deliberately NOT in the default
# set: those pins are humble branches and are ported separately.
DEFAULT_PKGS=(
  obstacle_detector
  dpcbf_viz_msgs
  sim_msgs
  dpcbf_ros_adapter
  safety_obstacle_filter
  g1_perception_utils
  g1_description
  sim_mjlidar_bridge
  g1_perception_bringup
  dpcbf_plot_client
)

docker_run() {
  local interactive=()
  [[ -t 0 ]] && interactive=(-it)
  docker run --rm "${interactive[@]}" \
    -v "${REPO_ROOT}:${REPO_ROOT}" \
    -w "${WS}" \
    -e "COLCON_HOME=${WS}/.colcon_foxy" \
    "${IMAGE}" "$@"
}

# colcon invocation with the Foxy-isolated bases, run under a foxy overlay.
colcon_in() {
  local verb="$1"; shift
  # NOTE: no `set -u` around the sourcing — Foxy's setup.bash reads
  # AMENT_TRACE_SETUP_FILES unguarded and would abort under nounset.
  docker_run bash -lc "
    set -eo pipefail
    source /opt/ros/foxy/setup.bash
    colcon --log-base log_foxy ${verb} \
      --build-base build_foxy --install-base install_foxy \
      $*
  "
}

cmd="${1:-shell}"; shift || true

case "${cmd}" in
  build-image)
    docker build \
      --build-arg "HOST_UID=$(id -u)" \
      --build-arg "HOST_GID=$(id -g)" \
      --build-arg "HOST_USER=$(id -un)" \
      -t "${IMAGE}" \
      "${WS}/docker/foxy"
    ;;

  shell)
    docker_run bash
    ;;

  exec)
    docker_run bash -lc "source /opt/ros/foxy/setup.bash && $*"
    ;;

  build)
    pkgs=("$@")
    # `build all` = the whole workspace including src/external, which is the
    # robot's real closure. `build` alone = only what this repo owns.
    if [[ "${pkgs[0]:-}" == "all" ]]; then
      # rmw_cyclonedds_cpp is skipped, not built: the pinned source is the
      # humble branch, which needs rmw headers Foxy does not have, and the foxy
      # branch needs CycloneDDS 0.7 while this workspace pins 0.10.2 for
      # unitree_sdk2. Foxy therefore uses the ros-foxy-rmw-cyclonedds-cpp deb
      # from the image. See the Dockerfile and README for the R-3 consequence.
      colcon_in build \
        "--packages-skip rmw_cyclonedds_cpp" \
        "--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo" \
        --continue-on-error --event-handlers console_direct+
    else
      [[ ${#pkgs[@]} -eq 0 ]] && pkgs=("${DEFAULT_PKGS[@]}")
      colcon_in build \
        "--packages-up-to ${pkgs[*]}" \
        "--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo" \
        --event-handlers console_direct+
    fi
    ;;

  test)
    pkgs=("$@")
    [[ ${#pkgs[@]} -eq 0 ]] && pkgs=("${DEFAULT_PKGS[@]}")
    colcon_in test "--packages-select ${pkgs[*]}" --event-handlers console_direct+
    docker_run bash -lc "
      source /opt/ros/foxy/setup.bash
      colcon --log-base log_foxy test-result --test-result-base build_foxy --verbose
    "
    ;;

  *)
    echo "usage: $0 {build-image|shell|build|test|exec} [args]" >&2
    exit 2
    ;;
esac
