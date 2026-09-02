from realtime.verify_config import DEFAULT_VERIFY, build_appe_cfg


def test_explicit_null_thresholds_use_fixed_operational_defaults():
    _, verify = build_appe_cfg({
        "verify": {"enabled": True, "mask_iou_min": None,
                   "texture_min_score": None, "iou_min": None}
    })
    assert verify["mask_iou_min"] == DEFAULT_VERIFY["mask_iou_min"]
    assert verify["texture_min_score"] == DEFAULT_VERIFY["texture_min_score"]
    assert verify["iou_min"] == DEFAULT_VERIFY["mask_iou_min"]
