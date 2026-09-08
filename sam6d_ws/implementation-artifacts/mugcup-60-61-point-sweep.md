# Mugcup frame 60/61 and coarse-point investigation

## Confirmed findings

- `output/longcircle2_sam_pem_full100_diagnostic/off.jsonl` contains Mugcup detections at `i=60` and `i=61`.
- Frame 60 selects proposal 5376 with `t=[-289.33,-27.35,815.63]` mm and fails the size gate (`size_ratio=1.01773`); frame 61 selects proposal 5124 with `t=[-295.03,-26.92,823.00]` mm and passes (`size_ratio=0.99171`). Their rotations differ substantially, which explains the handle facing change; this is a pose-hypothesis change, not missing handle geometry.
- The legacy verifier samples `coarse_npoint: 196` points in `config/base.yaml`; the resulting nearest-neighbour geometric/depth denominator is therefore evaluated on 196 observed points.

## Parameter decision

The production coarse point count remains **196**. A temporary 392-point experiment was executed separately, but it is not the production setting. The explorer's CAD proxy sampling remains `[::8]` to match production evidence density.

## Verification plan

Run the standard PEM inference/evaluation with `config/base.yaml`, compare frame-60/61 handle orientation, geometric rank stability, depth residuals, and runtime/VRAM against a 196-point baseline. Keep 196 if the 392-point run regresses accepted accuracy or exceeds the deployment budget.
