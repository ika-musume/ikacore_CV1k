#!/usr/bin/env bash
# r5 ship seed sweep: fit + cone/hist panels per seed.  Seed 3 (fit #7)
# is already done - sweep the rest, then restore SEED 3.
set -uo pipefail
cd "$(dirname "$0")"
for S in 1 2 5 7 10; do
    sed -i "s/^set_global_assignment -name SEED .*/set_global_assignment -name SEED $S/" ikacore_CV1k.qsf
    docker run --rm -u $(id -u):$(id -g) -e HOME=/host -v "$PWD":/host -w /host \
        raetro/quartus:17.0 quartus_sh --flow compile ikacore_CV1k \
        > fit_r5sweep_s$S.log 2>&1
    docker run --rm -u $(id -u):$(id -g) -e HOME=/host -v "$PWD":/host -w /host \
        raetro/quartus:17.0 quartus_sta -t sta_r5hist.tcl \
        > sta_r5sweep_s${S}_hist.log 2>&1
    echo "== seed $S done: $(grep -m1 'Worst-case setup' fit_r5sweep_s$S.log)"
done
sed -i "s/^set_global_assignment -name SEED .*/set_global_assignment -name SEED 3/" ikacore_CV1k.qsf
echo "== sweep complete (qsf restored to SEED 3) =="
