#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "vcortex/domain/settings.hpp"
#include "vcortex/io/map_store.hpp"
#include "vcortex/model/model.hpp"

namespace vcortex {

// one model run, already written in every format
struct ModelOutput {
    std::uint32_t run_id = 0, seq = 0;
    ModelReport report;
    std::map<std::string, std::string> by_format;  // connector id -> bytes
};

// one thread running run_model and every connector when the map store changed
// wakes on period, not per delta: a burst costs one run; new settings wake it at once
class ModelWorker {
public:
    // called on the worker thread after each run; a slow listener delays the next run
    using Listener = std::function<void(const std::shared_ptr<const ModelOutput>&)>;

    /// Creates the worker without starting its thread; on_output is called after each run.
    ModelWorker(MapStore& store, ModelSettings settings, double period_s, Listener on_output);
    /// Stops the thread before the worker goes away.
    ~ModelWorker();
    ModelWorker(const ModelWorker&) = delete;
    ModelWorker& operator=(const ModelWorker&) = delete;

    /// Starts the worker thread. Kept out of the constructor because the listener
    /// may use its owner, which is still being built.
    void start();
    /// Stops the worker thread and waits for it to end.
    void stop();

    /// Replaces the settings, already checked by apply_settings, and runs the model again on the same map.
    void set_settings(const ModelSettings& settings);
    /// Returns a copy of the current settings.
    ModelSettings settings() const;

    /// Returns the last output, shared and not copied, so a download and a broadcast read the same bytes.
    std::shared_ptr<const ModelOutput> latest() const;

    /// Forgets the last output and drops the run that may still be working on the old map.
    void clear();

private:
    /// Runs the model once at start, then again on each period or wake-up, until stop.
    void loop();
    /// Runs the model and every connector on the map, if the map changed since the last run.
    void run_once(const ModelSettings& settings);

    MapStore& store_;
    const std::chrono::duration<double> period_;
    const Listener on_output_;

    mutable std::mutex mutex_;
    std::condition_variable wake_cv_;
    ModelSettings settings_;
    std::shared_ptr<const ModelOutput> latest_;
    std::uint64_t generation_ = 0;  // bumped by clear(): a run started before it is dropped
    bool wake_ = false, stop_ = false;
    std::thread thread_;
};

}  // namespace vcortex
