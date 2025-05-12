
#include "motordatacollector.h"
#include <chrono>

MotorDataCollector::MotorDataCollector(MotorControl *controller,
                                       const std::vector<MotorParam> &paramList, int sampleCount,
                                       int intervalMs)
: controller_(controller),
  paramList_(paramList),
  intervalMs_(intervalMs),
  running_(false)
{
    sampleCount_ = std::max(0, std::min(sampleCount, MotorDataCollector::MAX_SAMPLE_COUNT));
}

MotorDataCollector::~MotorDataCollector() { stop(); }

void MotorDataCollector::start()
{
    if (running_) return;
    running_ = true;
    worker_ = std::thread(&MotorDataCollector::run, this);
}

void MotorDataCollector::stop()
{
    running_ = false;
    if (worker_.joinable()) worker_.join();
}

void MotorDataCollector::removeUpdateListeners() {
    std::lock_guard<std::mutex> lock(dataMutex_);
    onUpdate_.clear();
}

MotorData MotorDataCollector::getLatestData() const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    if (_data.empty())
        return MotorData{};
    return _data.back();
}

std::deque<MotorData> MotorDataCollector::getAllData() const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    return _data;
}

void MotorDataCollector::clearData() {
    std::lock_guard<std::mutex> lock(dataMutex_);
    _data.clear();
}

std::deque<MotorData> MotorDataCollector::getRecentData(int n) const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    if (n <= 0 || _data.empty())
        return {};

    int count = std::min(n, static_cast<int>(_data.size()));
    return std::deque<MotorData>(_data.end() - count, _data.end());
}

void MotorDataCollector::addUpdateListener(MotorDataCollectorUpdateCallback cb)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    onUpdate_.push_back(std::move(cb));
}

void MotorDataCollector::run()
{
    while (running_)
    {
        int64_t nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();

        auto raw = controller_->getMotorParams(1, paramList_);

        MotorData sample;
        sample.timestampMS = nowMs;
        sample.data_ = std::move(raw);

        {
            std::lock_guard<std::mutex> lock(dataMutex_);

            if (_data.size() >= static_cast<size_t>(sampleCount_)) _data.pop_front();

            _data.push_back(sample);

            for (const auto &cb : onUpdate_)
            {  // call listenercallback..
                cb(sample);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs_));
    }
}