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

    ModelWorker(MapStore& store, ModelSettings settings, double period_s, Listener on_output);
    ~ModelWorker();
    ModelWorker(const ModelWorker&) = delete;
    ModelWorker& operator=(const ModelWorker&) = delete;

    // not in the constructor: the listener may use its owner, which is still being built
    void start();
    void stop();

    // checked settings only (apply_settings); runs again on the same map
    void set_settings(const ModelSettings& settings);
    ModelSettings settings() const;

    // shared, not copied: a download and a broadcast read the same bytes
    std::shared_ptr<const ModelOutput> latest() const;

private:
    void loop();
    void run_once(const ModelSettings& settings);

    MapStore& store_;
    const std::chrono::duration<double> period_;
    const Listener on_output_;

    mutable std::mutex mutex_;
    std::condition_variable wake_cv_;
    ModelSettings settings_;
    std::shared_ptr<const ModelOutput> latest_;
    bool wake_ = false, stop_ = false;
    std::thread thread_;
};

}  // namespace vcortex
