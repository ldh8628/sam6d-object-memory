#!/usr/bin/env python3
import json,sys
from pathlib import Path
for root in sys.argv[1:]:
    for path in sorted(Path(root).glob('*/summary.json')):
        s=json.loads(path.read_text()); a=path.parent/'accuracy.json';a=json.loads(a.read_text()) if a.exists() else {}
        print(path.parent.name,'frames',s.get('processed_frames'),'valid',s.get('valid_poses'),
              'p99',s.get('valid_enqueue_to_publish_p99_ms'),'ATE',a.get('ate_rmse_m'),'rotation',a.get('fast_rotation_rpe_rmse_deg'))
