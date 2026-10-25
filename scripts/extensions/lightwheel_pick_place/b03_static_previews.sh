#!/usr/bin/env bash
set -eu
cd /root/autodl-tmp/RoboLaw-lgd
for variant in property human robot; do
bash scripts/extensions/lightwheel_pick_place/run_host.sh layout_preview_static.py --task kitchen_bench_lightwheel_place_bananas_in_wooden_bowl_u_shaped_with_island_farmhouse2 --variant "$variant" --variant-config configs/extensions/lightwheel_pick_place/variants_bananas_review_layout_v2.json --fixed-layout configs/extensions/lightwheel_pick_place/fixed_layout_bananas.json --display-layout configs/extensions/lightwheel_pick_place/fixed_layout_bananas_settled_review.json --seed 43 --placement_seed 42 --device cpu --output "/root/autodl-tmp/B03-review/${variant}_static" > "/root/autodl-tmp/lgd-b03-logs/${variant}_static.log" 2>&1
done
