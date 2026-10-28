#pragma once

namespace vcortex {

// TODO: the map, the trajectory, run_id, seq and a dirty flag behind one std::mutex.
// snapshot() copies the points out, so the model runs without holding the lock and ingest
// never waits for a model run.
class MapStore {};

}  // namespace vcortex
