/*
 * Author  : thep
 * Created : 2024
 * License : MIT
 *
 * Copyright (C) 2024 thep
 * SPDX-License-Identifier: MIT
 */

#include <thread>
#include <atomic>
#include <mutex>
#include <map>
#include <vector>
#include <deque>
#include <functional>
#include "motor-control/motor-control.hpp"

struct MotorData {
    int64_t timestampMS; // epochtime으로..
    std::map<MotorParam, std::vector<double>> data_;
};

using MotorDataCollectorUpdateCallback = std::function<void(const MotorData&)>;

class MotorDataCollector
{
public:
    static constexpr int MAX_SAMPLE_COUNT = 2048;
    MotorDataCollector(MotorControl *controller, const std::vector<MotorParam> &paramList,
                       int sampleCount, int intervalMs);
    ~MotorDataCollector();

    void start();
    void stop();

    void addUpdateListener(MotorDataCollectorUpdateCallback cb);
    void removeUpdateListeners();
    
    void clearData();
    void removeUpdateListeners();
    MotorData getLatestData() const;
    std::deque<MotorData> getAllData() const;
    std::deque<MotorData> getRecentData(int n) const;

private:
    void run();

    MotorControl *controller_;
    std::vector<MotorParam> paramList_;
    int sampleCount_;
    int intervalMs_;

    std::atomic<bool> running_;
    std::thread worker_;
    mutable std::mutex dataMutex_;
    std::deque<MotorData> _data;

    std::vector<MotorDataCollectorUpdateCallback> onUpdate_;
};