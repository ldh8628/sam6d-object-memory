# Story 0 Dev Story Result

## Story Title

Independent `object_memory` baseline reconstruction + SAM_circle bag usability
check (Story 0).

## Implementation Location

`objectmemory_ws/object_memory/` (this machine's workspace, replacing the
guide's `~/git_ws/object_memory/` path).

## Result Summary

Reconstructed the full Story 0 baseline from markdown only, with no prior code
present on this PC. Core data contracts, transform utilities, lifecycle state
machine, in-memory memory store, and the association contracts are implemented
and unit-tested. The SAM_circle bag was checked against real data.

- `compileall`: PASS.
- `pytest scripts_test -q`: 31 passed.
- bag checker: exit 0.

## SAM_circle Verdict

`부분 사용 가능` — usable as an RGB-D/IMU timestamp/source-data validation bag;
no SLAM pose or SAM-6D object output topics.

## Validation Commands

```bash
cd objectmemory_ws/object_memory
python3 -m compileall src scripts_test
python3 -m pytest scripts_test -q
python3 scripts_test/check_sam_circle_bag.py \
  --bag-path ~/temp_ws/CLI_environment/data_slam/rgbd_imu_sdk_bag/SAM_circle \
  --output-report reports/story0_sam_circle_bag_check.md \
  --max-topic-samples 5
```

## File List

- `pyproject.toml`, `README.md`
- `src/core/transforms.py`, `state_machine.py`, `models.py`,
  `memory_store.py`, `association_contracts.py`, `__init__.py`
- `src/object_memory/__init__.py`
- `scripts_test/check_sam_circle_bag.py`, `test_transforms.py`,
  `test_state_machine.py`, `test_memory_store.py`,
  `test_sam_circle_bag_contract.py`
- `reports/story0_sam_circle_bag_check.md`,
  `story0_object_memory_core_report.md`, `story0-dev-story-result.md`

## Dev Notes

- Path adaptation: the guide's default bag path
  (`~/ros2_bag_recording/output/rgbd_imu_sdk/SAM_circle`) does not exist on this
  PC. The real bag lives at
  `data_slam/rgbd_imu_sdk_bag/SAM_circle` and was passed explicitly via
  `--bag-path`. The script keeps the guide default so the contract test can
  verify the missing path is not created.
- `ros2` is not on PATH; the checker used its sqlite `.db3` fallback, exactly
  the sanctioned Story 0 path.
- `update_landmark` cannot receive `object_id` in `**changes` (bound by the
  signature), so identity immutability is enforced structurally; the reachable
  guards (unknown field, direct status change) are tested instead.
- Test count is 31 vs the guide's 21 target — a superset covering the same
  required cases plus extra validation (score range, coerce_status, missing
  path).

## Next Recommended Story Order

1. Story 1: Common SLAM Pose Adapter (start with ORB-SLAM3).
2. Story 2: SAM-6D Detection Output Contract.
3. Story 3: Object Memory Schema and Lifecycle.
4. Story 4: Short-term Association MVP.
5. Story 5: Long-term Object Memory Association MVP.
6. Story 6: Unity Object Update Contract.
7. Story 7: Diagnostics and Debug Report.
