SELF="$(readlink -f "${BASH_SOURCE[0]}")"
WS="${ASEP_WS:-$(cd "$(dirname "$SELF")/../../.." && pwd)}"

source /opt/ros/jazzy/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

# Keep each boat's ROS graph on its own Pi.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

case "$(hostname)" in
    boatoki)  export BOAT_ID=1 ;;
    floatoki) export BOAT_ID=2 ;;
    moatoki)  export BOAT_ID=3 ;;
    *)        unset BOAT_ID
              echo "env.sh: unknown hostname '$(hostname)', BOAT_ID not set" >&2 ;;
esac