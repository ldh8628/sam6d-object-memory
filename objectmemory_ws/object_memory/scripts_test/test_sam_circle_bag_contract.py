"""Contract tests for the SAM_circle bag checker.

These tests never create fake bag/dataset/SAM-output folders. When the default
bag path is absent, they assert the checker degrades to '사용 불가' without
materializing the missing path.
"""

import os

import check_sam_circle_bag as checker

VALID_VERDICTS = {
    checker.VERDICT_OK,
    checker.VERDICT_PARTIAL,
    checker.VERDICT_UNUSABLE,
}


def test_default_bag_check_returns_valid_schema():
    default_path = os.path.expanduser(checker.DEFAULT_BAG_PATH)
    result = checker.check_bag(checker.DEFAULT_BAG_PATH, max_topic_samples=3)

    # schema keys always present regardless of bag availability.
    for key in (
        "bag_path",
        "resolved_bag_dir",
        "folder_exists",
        "metadata_exists",
        "ros2_bag_info_ok",
        "verdict",
        "reasons",
        "topics",
        "created_fake_folders",
    ):
        assert key in result, key

    assert result["verdict"] in VALID_VERDICTS
    assert result["created_fake_folders"] is False

    if not os.path.isdir(default_path):
        # missing path must not be usable and must not be created.
        assert result["folder_exists"] is False
        assert result["verdict"] == checker.VERDICT_UNUSABLE
        assert not os.path.exists(default_path), "checker must not create the bag path"
    else:
        # real bag present -> RGB-D source, so at least partially usable.
        assert result["verdict"] in (checker.VERDICT_OK, checker.VERDICT_PARTIAL)
        assert isinstance(result["topics"], list)


def test_nonexistent_path_is_not_created():
    fake = os.path.join(
        os.path.dirname(__file__), "__definitely_missing_bag__", "SAM_circle"
    )
    assert not os.path.exists(fake)
    result = checker.check_bag(fake)
    assert result["verdict"] == checker.VERDICT_UNUSABLE
    assert result["folder_exists"] is False
    assert not os.path.exists(fake), "checker must not create missing bag path"
