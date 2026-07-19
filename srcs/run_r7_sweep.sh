#!/usr/bin/env bash
# r7 parallel seed farm: seeds {1 2 3 5 10} in scratch copies.
# Each copy: full compile + r5hist domain panels.
set -uo pipefail
cd "$(dirname "$0")"
FARM=/tmp/claude-1000/r7farm
mkdir -p "$FARM"
for S in 1 2 3 5 10; do
    D="$FARM/s$S"
    rm -rf "$D"; mkdir -p "$D"
    rsync -a --exclude db --exclude incremental_db --exclude output_files \
        ./ "$D/" > /dev/null
    sed -i "s/^set_global_assignment -name SEED .*/set_global_assignment -name SEED $S/" "$D/ikacore_CV1k.qsf"
    ( docker run --rm -u $(id -u):$(id -g) -e HOME=/host -v "$D":/host -w /host \
        raetro/quartus:17.0 quartus_sh --flow compile ikacore_CV1k \
        > "$D/fit.log" 2>&1
      docker run --rm -u $(id -u):$(id -g) -e HOME=/host -v "$D":/host -w /host \
        raetro/quartus:17.0 quartus_sta -t sta_r5hist.tcl \
        > "$D/hist.log" 2>&1
      echo "== seed $S: $(grep -m1 'Worst-case setup' "$D/fit.log" || echo FIT-FAILED)" ) &
done
wait
echo "== r7 sweep complete =="
for S in 1 2 3 5 10; do
    c153=$(grep -m1 "Worst case slack" "$FARM/s$S/hist.log" | awk '{print $NF}')
    c102=$(grep "Worst case slack" "$FARM/s$S/hist.log" | sed -n '2p' | awk '{print $NF}')
    echo "seed $S: c153 $c153 / c102 $c102"
done
