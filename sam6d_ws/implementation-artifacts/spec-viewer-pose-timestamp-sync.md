---
title: 'Viewer pose timestamp synchronization'
type: 'bugfix'
created: '2026-09-04'
status: 'done'
route: 'one-shot'
---

# Viewer pose timestamp synchronization

## Intent

**Problem:** The map viewer updated SLAM and transformed SAM camera markers independently, so poses from adjacent timestamps could appear together during motion.

**Approach:** Update both markers only from an exact timestamp pair, while retaining the previous independent mode behind `SAM6D_VIEWER_POSE_SYNC=0` for immediate comparison or rollback.

## Suggested Review Order

- Exact pairing is default; the environment switch preserves the previous behavior.
  [`sam6d_viewer.py:198`](../realtime/sam6d_viewer.py#L198)

- A matched pair updates both displayed camera markers in one callback.
  [`sam6d_viewer.py:221`](../realtime/sam6d_viewer.py#L221)

- The self-check rejects a one-nanosecond mismatch before accepting an exact pair.
  [`sam6d_viewer.py:281`](../realtime/sam6d_viewer.py#L281)
