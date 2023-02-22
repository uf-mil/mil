#!/bin/bash
# Script to run the propriatary sylphase driver at the specified port
set -euo pipefail

usage() {
	echo "Usage: port"
	echo "Example ./run_sylphase_driver 10001"
	exit 1
}
# Call usage() function if arguments not supplied.
[[ $# -lt 1 ]] && usage

# Run sylphase
exec "$MIL_CONFIG_DIR"/sylphase-sonar/driver/publish "$@"
