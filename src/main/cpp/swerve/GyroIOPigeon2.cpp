

#include "swerve/GyroIOPigeon2.h"
#include "swerve/TalonOdometryThread.h"

GyroIOPigeon2::GyroIOPigeon2( int canId, std::string canBus ) : pigeon{ canId, canBus }
{
    ctre::phoenix6::configs::Pigeon2Configuration config;
    pigeon.GetConfigurator().Apply( config );
    pigeon.GetConfigurator().SetYaw( 0.0_rad );

    yaw.SetUpdateFrequency( TalonOdometryThread::ODOMETRY_FREQUENCY );
    yawVelocity.SetUpdateFrequency( 100_Hz );
    pigeon.OptimizeBusUtilization();

    yawTimestampQueue = TalonOdometryThread::GetInstance()->MakeTimestampQueue();
    yawPositionQueue = TalonOdometryThread::GetInstance()->RegisterSignal( pigeon, &yaw );
}

void GyroIOPigeon2::UpdateInputs(Inputs &inputs)
{
    inputs.connected = ctre::phoenix6::BaseStatusSignal::RefreshAll( yaw, yawVelocity ) == ctre::phoenix::StatusCode::OK;
    inputs.yawPosition = yaw.GetValue();
    inputs.yawVelocity = yawVelocity.GetValue();

    inputs.odometryYawTimestamps.clear();
    for (; !yawTimestampQueue->empty(); yawTimestampQueue->pop()) {
        inputs.odometryYawTimestamps.push_back(yawTimestampQueue->front());
    }

    inputs.odometryYawPositions.clear();
    for (; !yawPositionQueue->empty(); yawPositionQueue->pop()) {
        inputs.odometryYawPositions.push_back(yawPositionQueue->front() * 1_deg);
    }
}