/*
 * Author  : thep
 * Created : 2024
 * License : MIT
 *
 * Copyright (C) 2024 thep
 * SPDX-License-Identifier: MIT
 */

#include <unistd.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include "motor-control/motor-control.hpp"
#include "motordatacollector.h"

using namespace std;

int main()
{
    auto motor = std::unique_ptr<MotorControl>(MotorControl::getMotorControlInstance(1234));

    motor->setOperationMode(MotorOpMode::kModeOpenLoop);

    vector<MotorParam> params = {MotorParam::kVoltagePhaseA, //로깅할 parameter들
                                 MotorParam::kCurrentPhaseA,
                                 MotorParam::kVoltagePhaseB,
                                 MotorParam::kCurrentPhaseB,
                                 MotorParam::kVoltagePhaseC,
                                 MotorParam::kCurrentPhaseC,
                                 MotorParam::kId,
                                 MotorParam::kIq,
                                 MotorParam::kIalpha,
                                 MotorParam::kIbeta,
                                 MotorParam::kRpm,
                                 MotorParam::kPosition};

    MotorDataCollector collector(motor, params, 256, 1000);

    collector.addUpdateListener(
        [](const MotorData &sample)
        {
            std::time_t t = sample.timestampMS / 1000;
            int ms = sample.timestampMS % 1000;
            std::tm tm = *std::localtime(&t);

            std::cout << "[" << std::put_time(&tm, "%F %T") << "." << std::setw(3)
                      << std::setfill('0') << ms << "]\n";

            for (const auto &[param, values] : sample.data_)
            {
                std::cout << "  Param " << static_cast<int>(param) << ": ";
                for (auto v : values) std::cout << v << " ";
                std::cout << std::endl;
            }
            std::cout << std::endl;
        });

    collector.start();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    collector.stop();

    auto allData = collector.getAllData();
    std::cout << "Recorded: " << allData.size() << "\n";

    motor->setOperationMode(MotorOpMode::kModeOff);
    delete motor;

    return 0;
}
