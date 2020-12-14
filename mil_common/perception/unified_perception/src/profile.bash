#!/bin/bash
set -euo pipefail
CURR="$(realpath $(dirname $BASH_SOURCE))"
python -m cProfile -o $CURR/capture.stats $CURR/unified_perception.py
