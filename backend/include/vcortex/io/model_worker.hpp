#pragma once

namespace vcortex {

// TODO: one thread that runs run_model and every connector when the map store is dirty.
// It wakes on a timer (model_period), not on every delta, so a burst of deltas costs one model
// run. New settings wake it at once, so a changed value shows without waiting a full period.
class ModelWorker {};

}  // namespace vcortex
