#include <atomic>
#include <cassert>
#include <thread>
#include "Atlas.h"

int main()
{
    ORB_SLAM3::Atlas empty;
    assert(empty.GetCurrentMapId() == -1);
    assert(empty.CountMaps() == 0); // Reading identity must not create a map.
    ORB_SLAM3::Atlas atlas(0);
    auto* first = atlas.GetCurrentMap();
    const auto first_id = atlas.GetCurrentMapId();
    assert(first_id >= 0);
    atlas.CreateNewMap();
    auto* second = atlas.GetCurrentMap();
    const auto second_id = atlas.GetCurrentMapId();
    assert(second_id != first_id);
    atlas.ChangeMap(first);
    assert(atlas.GetCurrentMapId() == first_id);
    std::atomic<bool> done{false};
    std::thread reader([&]() {
        while (!done.load()) {
            const auto id = atlas.GetCurrentMapId();
            assert(id == first_id || id == second_id);
        }
    });
    for (int i = 0; i < 100; ++i) {
        atlas.ChangeMap(i % 2 ? first : second);
    }
    done = true;
    reader.join();
    second->ChangeId(500);
    atlas.ChangeMap(second);
    assert(atlas.GetCurrentMapId() == 500);
    // Saving an uninitialized Atlas must not invalidate its live current map:
    // shutdown exports (map points / loop edges) still query it after PreSave.
    atlas.PreSave();
    assert(!second->IsBad());
    assert(atlas.GetCurrentMap() == second);
    assert(atlas.GetCurrentMapId() == 500);
    assert(atlas.CountMaps() == 2);
    atlas.PreSave();
    assert(atlas.GetCurrentMap() == second);
}
