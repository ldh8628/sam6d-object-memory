# Blind Hunter Review Prompt — Loop 1

Invoke the `bmad-review-adversarial-general` skill on this diff:

## Review target

- Baseline commit: `1f221f6a944fc0e9ece77d87792c55dbef67fac9`
- Approved spec: `_bmad-output/implementation-artifacts/spec-convert-0729-rgbd-bags.md`
- Tracked code changes:
  - `data_slam/260714_frame_data/convert_recording.py`
  - `data_slam/0724_chungbuk/make_orb_settings_sdk.py`
  - `data_slam/260714_frame_data/test_convert_recording_contract.py`
- Verification evidence:
  - `_bmad-output/implementation-artifacts/verification-convert-0729-rgbd-bags.md`
- Generated outputs:
  - `data_slam/0729/converted/0729_long_circle_inner/`
  - `data_slam/0729/converted/0729_smaill_circle_inner/`
  - `data_slam/0729/converted/0729_small_8/`
  - `data_slam/0729/settings/`

## Construct the textual diff

Run these read-only commands from the repository root:

```bash
git diff 1f221f6a944fc0e9ece77d87792c55dbef67fac9 -- \
  data_slam/260714_frame_data/convert_recording.py \
  data_slam/0724_chungbuk/make_orb_settings_sdk.py
sed -n '1,320p' \
  data_slam/260714_frame_data/test_convert_recording_contract.py
sed -n '1,320p' \
  _bmad-output/implementation-artifacts/spec-convert-0729-rgbd-bags.md
sed -n '1,320p' \
  _bmad-output/implementation-artifacts/verification-convert-0729-rgbd-bags.md
find data_slam/0729/converted data_slam/0729/settings \
  -maxdepth 3 -type f -printf '%p %s bytes\n' | sort
```

The original `data_slam/0729/0729_*` bags, BMad installation files, and
`node_modules/` predated implementation and are user-owned baseline state.
Do not treat them as patch additions. Do inspect their metadata read-only when
needed to verify the converter's assumptions.

Review the loop-1 implementation against every frozen constraint, task,
acceptance criterion, review change-log KEEP instruction, and verification
requirement. Pay particular attention to whether the previous findings are
actually closed without introducing regressions. Return only actionable
findings with severity, exact `path:line` evidence, consequence, and a concrete
correction or proof needed. Explicitly report when no finding survives
verification.
