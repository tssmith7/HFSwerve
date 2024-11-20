
#include <ctre/phoenix6/CANBus.hpp>

#include "swerve/TalonOdometryThread.h"


TalonOdometryThread* TalonOdometryThread::singleton = nullptr;
const units::hertz_t TalonOdometryThread::ODOMETRY_FREQUENCY = 250.0_Hz;

TalonOdometryThread* TalonOdometryThread::GetInstance() {
    if( singleton == nullptr ) {
        singleton = new TalonOdometryThread{};
    }

    return singleton;
}

std::queue<units::second_t>* TalonOdometryThread::MakeTimestampQueue() {
    odometryLock.lock();
    std::queue<units::second_t>* tsQueue = &timestampQueues.emplace_back( std::queue<units::second_t>{} );
    odometryLock.unlock();

    return tsQueue;
}

std::queue<double>* TalonOdometryThread::RegisterSignal( 
    ctre::phoenix6::hardware::ParentDevice& device, 
    ctre::phoenix6::BaseStatusSignal* signal) 
{
    signalsLock.lock();
    odometryLock.lock();
    isCANFD = ctre::phoenix6::CANBus::IsNetworkFD( device.GetNetwork() );
    signals.push_back( signal );
    std::queue<double>* newQ = &queues.emplace_back( std::queue<double>{} );
    odometryLock.unlock();
    signalsLock.unlock();

    return newQ;
}

void TalonOdometryThread::Start() {
    if( timestampQueues.size() > 0 ) {
        thread = std::thread( &TalonOdometryThread::Run, this );
    }
}


void TalonOdometryThread::Run() {

}

