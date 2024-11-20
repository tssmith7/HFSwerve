#pragma once

#include <queue>
#include <vector>
#include <thread>
#include <mutex>

#include <units/time.h>

#include <ctre/phoenix6/TalonFX.hpp>

class TalonOdometryThread {
public:
    static TalonOdometryThread* GetInstance();
    std::queue<units::second_t>* MakeTimestampQueue();
    std::queue<double>* RegisterSignal( 
        ctre::phoenix6::hardware::ParentDevice& device, 
        ctre::phoenix6::BaseStatusSignal* signal
    );
    void Start();

    static const units::hertz_t ODOMETRY_FREQUENCY;
    std::mutex odometryLock;

private:
    TalonOdometryThread() {}
    void Run();

    static TalonOdometryThread *singleton;
    std::mutex signalsLock;
    std::vector<ctre::phoenix6::BaseStatusSignal*> signals;
    std::vector<std::queue<units::second_t> *> timestampQueues;
    std::vector<std::queue<double> *> queues;
    std::thread thread;
    bool isCANFD;
};
