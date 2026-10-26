#pragma once

namespace vcortex {

// Step 15: one std::thread that wakes every model_period (or at once on new settings),
// runs run_model and every connector when the store is dirty, and pushes the outputs.
class ModelWorker {};

}  // namespace vcortex
