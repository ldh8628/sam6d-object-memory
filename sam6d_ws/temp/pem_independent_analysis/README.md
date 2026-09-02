# PEM geometry-only + independent verification

- detections: 6
- strict size failures (>1.00): 4
- tolerance failures (>1.05 / >1.10): 1 / 0
- mask IoU below .30/.40/.50: 0 / 1 / 6
- geometry winner texture top-1/top-5/top-10: 0 / 0 / 0

No validated SAM-camera GT is available; these are filter distributions, not accuracy measurements.

## Representative cases

```json
[
  {
    "stamp_ns": 1785831526041608960,
    "object": "choco_hazelnut_high",
    "status": "not_detected"
  },
  {
    "stamp_ns": 1785831518074359552,
    "object": "saffron",
    "status": "not_detected"
  },
  {
    "stamp_ns": 1785831536184341504,
    "object": "saffron",
    "status": "not_detected"
  }
]
```
