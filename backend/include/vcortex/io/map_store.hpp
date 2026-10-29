#pragma once

namespace vcortex {

// TODO: map, trajectory, run_id, seq and dirty flag behind one std::mutex
// snapshot() copies points out: the model runs unlocked, ingest never waits for it
class MapStore {};

}  // namespace vcortex
