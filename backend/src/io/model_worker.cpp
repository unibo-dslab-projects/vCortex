#include "vcortex/io/model_worker.hpp"

#include <exception>
#include <iostream>
#include <optional>
#include <utility>

#include "vcortex/connectors/connector.hpp"

namespace vcortex {

ModelWorker::ModelWorker(MapStore& store, ModelSettings settings, double period_s,
                         Listener on_output)
    : store_(store),
      period_(period_s),
      on_output_(std::move(on_output)),
      settings_(std::move(settings)) {}

ModelWorker::~ModelWorker() { stop(); }

void ModelWorker::start() {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (thread_.joinable()) return;
    stop_ = false;
    thread_ = std::thread([this] { loop(); });
}

void ModelWorker::stop() {
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    wake_cv_.notify_all();
    if (thread_.joinable()) thread_.join();
}

void ModelWorker::set_settings(const ModelSettings& settings) {
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        settings_ = settings;
        wake_ = true;
    }
    store_.touch();
    wake_cv_.notify_all();
}

ModelSettings ModelWorker::settings() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return settings_;
}

std::shared_ptr<const ModelOutput> ModelWorker::latest() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
}

// checks once before the first wait, so offline mode has a model at start, not after a period
void ModelWorker::loop() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!stop_) {
        const ModelSettings settings = settings_;
        lock.unlock();
        run_once(settings);
        lock.lock();
        // settings sent during a run set wake_, so the next run follows at once: never more
        // than one run waiting
        wake_cv_.wait_for(lock, period_, [this] { return stop_ || wake_; });
        wake_ = false;
    }
}

void ModelWorker::run_once(const ModelSettings& settings) {
    std::optional<MapCopy> copy = store_.take();
    if (!copy) return;
    // an exception leaving the thread would end the whole server; a failed run only loses
    // this output, and the next change runs again
    try {
        const ModelResult result = run_model(copy->snapshot, copy->trajectory, settings);
        auto out = std::make_shared<ModelOutput>();
        out->run_id = result.building.run_id;
        out->seq = result.building.seq;
        out->report = result.report;
        for (const Connector& c : connectors()) {
            try {
                out->by_format.emplace(c.id, c.write(result));
            } catch (const std::exception& e) {
                std::cerr << "model worker: " << c.id << " failed: " << e.what() << '\n';
            }
        }
        std::shared_ptr<const ModelOutput> done = std::move(out);
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            latest_ = done;
        }
        if (on_output_) on_output_(done);
    } catch (const std::exception& e) {
        std::cerr << "model worker: run failed: " << e.what() << '\n';
    }
}

}  // namespace vcortex
