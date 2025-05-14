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
#include "epcl/epcl_variables.h"
#include "motor-control/motor-control.hpp"
#include "motordatacollector.h"

using namespace std;

vector<MotorParam> paramsToMonitor = {MotorParam::kVoltagePhaseA,
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

auto paramUpdateHandler = [](const MotorData &sample)
{
    std::time_t t = sample.timestampMS / 1000;
    int ms = sample.timestampMS % 1000;
    std::tm tm = *std::localtime(&t);

    std::cout << "[" << std::put_time(&tm, "%F %T") << "." << std::setw(3) << std::setfill('0')
              << ms << "]\n";

    for (const auto &[param, values] : sample.data_)
    {
        std::cout << "  Param " << static_cast<int>(param) << ": ";
        for (auto v : values) std::cout << v << " ";
        std::cout << std::endl;
    }
    std::cout << std::endl;
};

int main()
{
    auto mc = std::unique_ptr<MotorControl>(MotorControl::getMotorControlInstance(1234));

    if (!mc)
    {
        std::cout << "mc == Null" << std::endl;
        std::abort();
    }

    mc->setOperationMode(MotorOpMode::kModeOpenLoop);

    MotorDataCollector collector(mc, paramsToMonitor, 256, 1000);
    collector.addUpdateListener(paramUpdateHandler);

    collector.start();

    std::vector<std::thread> jobs;
    try
    {
        jobs.emplace_back(worker);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        for (auto &t : jobs)
        {
            if (t.joinable()) t.join();
        }
    }
    catch (...)
    {
        collector.stop();
        std::cout << "failded to emplace" << std::endl;
        std::abort();
    }
    collector.stop();

    auto allData = collector.getAllData();
    std::cout << "Recorded: " << allData.size() << "\n";

    mc->setOperationMode(MotorOpMode::kModeOff);
    delete mc;

    return 0;
}

void worker()
{
    int t = 5;
    while (t-- > 0)
    {
        std::cout << "now t =" << t << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(15));
    }
    return;
}
/*
void UpdateCC(const MotorParam &param)
{
    float Wc;
    float Ra;

    float Kpd = param.Lds * Wc;
    float Kid_T = (param.Rs + Ra) * Wc * Tsamp;
    float Kad = 1. / tmpC.Kpd;

    float Kpq = param.Lqs * Wc;
    float Kiq_T = (param.Rs + Ra) * Wc * Tsamp;
    float Kaq = 1. / tmpC.Kpq;

    float Krd = tmpC.Kr_Ratio * (param.Rs + Ra) * Wc;
    float Krq = tmpC.Kr_Ratio * (param.Rs + Ra) * Wc;
    // float Wres[0] = 6 * param.Wr;
    // float Wres[1] = 12 * param.Wr;
}
    */