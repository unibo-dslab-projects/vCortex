#pragma once

namespace vcortex {

// Steps 14-15: the backend's copy of the map, the trajectory, run_id, seq and a dirty flag,
// behind one std::mutex. Ingest handlers write to it; snapshot() copies the points out.
class MapStore {};

}  // namespace vcortex
