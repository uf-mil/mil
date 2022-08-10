#! /bin/bash
# A script to download logs generated by OSRF in 2019 VRX Phase 3


set -euo pipefail
aws s3 sync s3://vrx-events/2019/phase3_vrx_challenge . --no-sign-request --exclude "*" --include "*uf*"
