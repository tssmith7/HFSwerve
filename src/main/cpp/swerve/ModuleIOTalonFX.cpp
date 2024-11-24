
#include "swerve/ModuleIOTalonFX.h"
#include "swerve/TalonOdometryThread.h"

#include "swerve/SwerveConstants.h"

ModuleIOTalonFX::ModuleIOTalonFX( const ModuleConfigs& configs ) :
    index{ configs.index },
    m_driveMotor{ configs.driveCanId, configs.canBus },
    m_turnMotor{ configs.turnCanId, configs.canBus },
    m_encoder{ configs.encoderCanId, configs.canBus }
{
    {
        ctre::phoenix6::configs::TalonFXConfiguration driveConfigs{};
        driveConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        const TuningParams& tune = configs.driveTune.tuner;
        driveConfigs.Slot0.kP = tune.kP;
        driveConfigs.Slot0.kI = tune.kI;
        driveConfigs.Slot0.kD = tune.kD;
        driveConfigs.Slot0.kS = tune.kS;
        driveConfigs.Slot0.kG = tune.kG;
        driveConfigs.Slot0.kV = tune.kV;
        driveConfigs.Slot0.kA = tune.kA;

        driveConfigs.MotionMagic.MotionMagicCruiseVelocity = 
            units::turns_per_second_t(configs.driveTune.mp.MaxVelocity).value();            /* in rotations per second */
        driveConfigs.MotionMagic.MotionMagicAcceleration =
            units::turns_per_second_squared_t(configs.driveTune.mp.MaxAcceleration).value(); /* in rotations per second^2 */
        driveConfigs.MotionMagic.MotionMagicJerk =
            units::turns_per_second_cubed_t(configs.driveTune.mp.MaxJerk).value();          /* in rotations per second^3 */

        driveConfigs.Feedback.SensorToMechanismRatio = swerve::physical::kDriveGearRatio;
        m_driveMotor.GetConfigurator().Apply(driveConfigs);
        SetDriveBrakeMode( true );
    }

    {
        ctre::phoenix6::configs::TalonFXConfiguration turnConfigs{};
        turnConfigs.CurrentLimits.SupplyCurrentLimit = 30;
        turnConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        const TuningParams& tune = configs.turnTune.tuner;
        turnConfigs.Slot0.kP = tune.kP;
        turnConfigs.Slot0.kI = tune.kI;
        turnConfigs.Slot0.kD = tune.kD;
        turnConfigs.Slot0.kS = tune.kS;
        turnConfigs.Slot0.kG = tune.kG;
        turnConfigs.Slot0.kV = tune.kV;
        turnConfigs.Slot0.kA = tune.kA;

        turnConfigs.MotionMagic.MotionMagicCruiseVelocity = 
            units::turns_per_second_t(configs.turnTune.mp.MaxVelocity).value();            /* in rotations per second */
        turnConfigs.MotionMagic.MotionMagicAcceleration =
            units::turns_per_second_squared_t(configs.turnTune.mp.MaxAcceleration).value(); /* in rotations per second^2 */
        turnConfigs.MotionMagic.MotionMagicJerk =
            units::turns_per_second_cubed_t(configs.turnTune.mp.MaxJerk).value();          /* in rotations per second^3 */

        turnConfigs.Feedback.FeedbackRemoteSensorID = m_encoder.GetDeviceID();
        turnConfigs.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
        turnConfigs.Feedback.RotorToSensorRatio = swerve::physical::kTurnGearRatio;
        m_turnMotor.GetConfigurator().Apply(turnConfigs);
        SetTurnBrakeMode( true );
    }

    ctre::phoenix6::configs::CANcoderConfiguration encoderConfigs{};
    encoderConfigs.MagnetSensor.MagnetOffset = configs.absoluteEncoderOffset;
    m_encoder.GetConfigurator().Apply( encoderConfigs );

    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll( 
        TalonOdometryThread::ODOMETRY_FREQUENCY, 
        drivePosition, 
        turnPosition
    );
    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll( 
        50_Hz, 
        driveVelocity, 
        driveAppliedVolts, 
        driveCurrent,
        turnAbsolutePosition, 
        turnVelocity, 
        turnAppliedVolts, 
        turnCurrent
    );

    m_driveMotor.OptimizeBusUtilization();
    m_turnMotor.OptimizeBusUtilization();

    timestampQueue = TalonOdometryThread::GetInstance()->MakeTimestampQueue();
    drivePositionQueue = TalonOdometryThread::GetInstance()->RegisterSignal( m_driveMotor, &drivePosition );
    turnPositionQueue = TalonOdometryThread::GetInstance()->RegisterSignal( m_turnMotor, &turnPosition );
}

void ModuleIOTalonFX::UpdateInputs(Inputs& inputs) {
    ctre::phoenix6::BaseStatusSignal::RefreshAll(
        drivePosition, 
        driveVelocity, 
        driveAppliedVolts, 
        driveCurrent,
        turnPosition,
        turnAbsolutePosition, 
        turnVelocity, 
        turnAppliedVolts, 
        turnCurrent
    );

    inputs.drivePosition = drivePosition.GetValue();
    inputs.driveVelocity = driveVelocity.GetValue();
    inputs.driveAppliedVolts = driveAppliedVolts.GetValue();
    inputs.driveCurrent = driveCurrent.GetValue();

    inputs.turnAbsolutePosition = turnAbsolutePosition.GetValue();
    inputs.turnPosition = turnPosition.GetValue() / swerve::physical::kTurnGearRatio;
    inputs.turnVelocity = turnVelocity.GetValue() / swerve::physical::kTurnGearRatio;
    inputs.turnAppliedVolts = turnAppliedVolts.GetValue();
    inputs.turnCurrent = turnCurrent.GetValue();

    inputs.odometryTimestamps.clear();
    for (; !timestampQueue->empty(); timestampQueue->pop()) {
        inputs.odometryTimestamps.push_back(timestampQueue->front());
    }

    inputs.odometryDrivePositions.clear();
    for (; !drivePositionQueue->empty(); drivePositionQueue->pop()) {
        inputs.odometryDrivePositions.push_back(drivePositionQueue->front() * 1_rad);
    }

    inputs.odometryTurnPositions.clear();
    for (; !turnPositionQueue->empty(); turnPositionQueue->pop()) {
        inputs.odometryTurnPositions.push_back(turnPositionQueue->front() * 1_rad);
    }
}

void ModuleIOTalonFX::SetDriveOpenLoop( double percent ) {
    m_driveMotor.SetControl( ctre::phoenix6::controls::DutyCycleOut( percent ) );
}

void ModuleIOTalonFX::SetTurnOpenLoop( double percent ) {
    m_turnMotor.SetControl( ctre::phoenix6::controls::DutyCycleOut( percent ) );
}

void ModuleIOTalonFX::SetDriveWheelVelocity( units::radians_per_second_t velocity )
{
    m_driveMotor.SetControl( ctre::phoenix6::controls::MotionMagicVelocityVoltage( velocity ) );
}

void ModuleIOTalonFX::SetTurnPosition( units::radian_t position )
{
    m_turnMotor.SetControl( ctre::phoenix6::controls::MotionMagicVoltage( position ) );
}

void ModuleIOTalonFX::SetDriveBrakeMode( bool enable ) {
    ctre::phoenix6::configs::MotorOutputConfigs config{};
    config.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    config.NeutralMode = enable ? 
        ctre::phoenix6::signals::NeutralModeValue::Brake : ctre::phoenix6::signals::NeutralModeValue::Coast;
    m_driveMotor.GetConfigurator().Apply( config );
}

void ModuleIOTalonFX::SetTurnBrakeMode( bool enable ) {
    ctre::phoenix6::configs::MotorOutputConfigs config{};
    config.Inverted = isTurnMotorInverted ? 
        ctre::phoenix6::signals::InvertedValue::Clockwise_Positive :
        ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    config.NeutralMode = enable ? 
        ctre::phoenix6::signals::NeutralModeValue::Brake : 
        ctre::phoenix6::signals::NeutralModeValue::Coast;
    m_turnMotor.GetConfigurator().Apply( config );
}
