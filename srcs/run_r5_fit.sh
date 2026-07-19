#!/usr/bin/env bash
# r5 fit + panel: sync rtl/, compile, cone panel + r5 domain-wide probe.
#   ./run_r5_fit.sh <tag>            e.g. ./run_r5_fit.sh r5_1
set -uo pipefail
cd "$(dirname "$0")"
TAG="${1:?tag}"
../sim/scripts/sync_srcs.sh
docker run --rm -u $(id -u):$(id -g) -e HOME=/host -v "$PWD":/host -w /host \
    raetro/quartus:17.0 quartus_sh --flow compile ikacore_CV1k \
    > fit_${TAG}.log 2>&1
echo "== fit done: $(grep -m1 'Worst-case setup' fit_${TAG}.log || echo FIT-FAILED)"
docker run --rm -u $(id -u):$(id -g) -e HOME=/host -v "$PWD":/host -w /host \
    raetro/quartus:17.0 quartus_sta -t sta_cones.tcl > sta_${TAG}.log 2>&1
docker run --rm -u $(id -u):$(id -g) -e HOME=/host -v "$PWD":/host -w /host \
    raetro/quartus:17.0 quartus_sta -t sta_r5hist.tcl > sta_${TAG}_hist.log 2>&1
echo "== panels done: sta_${TAG}.log / sta_${TAG}_hist.log"
