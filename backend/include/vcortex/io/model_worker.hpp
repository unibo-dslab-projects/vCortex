#pragma once

namespace vcortex {

// TODO: one thread running run_model and every connector when the map store is dirty
// wakes on model_period, not per delta: a burst costs one run; new settings wake it at once
class ModelWorker {};

}  // namespace vcortex
