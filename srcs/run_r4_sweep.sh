#!/usr/bin/env bash
# r4 ship seed sweep: fit + cone panel per seed, panels kept as
# sta_r4sweep_s<N>.log.  Seed 2 (fit #7) is already done - sweep the rest.
set -uo pipefail
cd "$(dirname "$0")"
for S in 1 3 7 10; do
    sed -i "s/^set_global_assignment -name SEED .*/set_global_assignment -name SEED $S/" ikacore_CV1k.qsf
    docker run --rm -u $(id -u):$(id -g) -e HOME=/host -v "$PWD":/host -w /host \
        raetro/quartus:17.0 quartus_sh --flow compile ikacore_CV1k \
        > fit_r4sweep_s$S.log 2>&1
    docker run --rm -u $(id -u):$(id -g) -e HOME=/host -v "$PWD":/host -w /host \
        raetro/quartus:17.0 quartus_sta -t sta_cones.tcl \
        > sta_r4sweep_s$S.log 2>&1
    echo "== seed $S done: $(grep -m1 'Worst-case setup' fit_r4sweep_s$S.log)"
done
sed -i "s/^set_global_assignment -name SEED .*/set_global_assignment -name SEED 2/" ikacore_CV1k.qsf
echo "== sweep complete (qsf restored to SEED 2) =="
