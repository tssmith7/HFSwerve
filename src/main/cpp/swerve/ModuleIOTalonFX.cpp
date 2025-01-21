
#include "swerve/ModuleIOTalonFX.h"
#include "swerve/TalonOdometryThread.h"

ModuleIOTalonFX::ModuleIOTalonFX( ModuleConfigs configs ) 
    : m_driveMotor{ configs.driveCanId, configs.canBus },
      m_turnMotor{ configs.turnCanId, configs.canBus },
      m_encoder{ configs.encoderCanId, configs.canBus }
{
  ctre::phoenix6::CANBus canBus{ configs.canBus };
  ctre::phoenix6::configs::TalonFXConfiguration driveConfigs{};
  driveConfigs.CurrentLimits.SupplyCurrentLimit = 40_A;
  driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
  m_driveMotor.GetConfigurator().Apply(driveConfigs);
  setDriveBrakeMode( true );

  ctre::phoenix6::configs::TalonFXConfiguration turnConfigs{};
  turnConfigs.CurrentLimits.SupplyCurrentLimit = 30_A;
  turnConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
  m_turnMotor.GetConfigurator().Apply(turnConfigs);
  setTurnBrakeMode( true );

  ctre::phoenix6::configs::CANcoderConfiguration encoderConfigs{};
  encoderConfigs.MagnetSensor.MagnetOffset = configs.absoluteEncoderOffset;
  m_encoder.GetConfigurator().Apply( encoderConfigs );

  units::hertz_t odom_freq = TalonOdometryThread::RIO_ODOMETRY_FREQUENCY;
  if( canBus.IsNetworkFD() ) {
    odom_freq = TalonOdometryThread::CANFD_ODOMETRY_FREQUENCY;
  }
  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll( 
    odom_freq, 
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
  inputs.turnPosition = turnPosition.GetValue() / turnGearRatio;
  inputs.turnVelocity = turnVelocity.GetValue() / turnGearRatio;
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

void ModuleIOTalonFX::setDriveVoltage( units::volt_t volts ) {
  m_driveMotor.SetControl( ctre::phoenix6::controls::VoltageOut( volts ) );
}

void ModuleIOTalonFX::setTurnVoltage( units::volt_t volts ) {
  m_turnMotor.SetControl( ctre::phoenix6::controls::VoltageOut( volts ) );
}

void ModuleIOTalonFX::setDriveBrakeMode( bool enable ) {
  ctre::phoenix6::configs::MotorOutputConfigs config{};
  config.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  config.NeutralMode = enable ? 
    ctre::phoenix6::signals::NeutralModeValue::Brake : ctre::phoenix6::signals::NeutralModeValue::Coast;
  m_driveMotor.GetConfigurator().Apply( config );
}

void ModuleIOTalonFX::setTurnBrakeMode( bool enable ) {
  ctre::phoenix6::configs::MotorOutputConfigs config{};
  config.Inverted = isTurnMotorInverted ? 
    ctre::phoenix6::signals::InvertedValue::Clockwise_Positive :
    ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  config.NeutralMode = enable ? 
    ctre::phoenix6::signals::NeutralModeValue::Brake : 
    ctre::phoenix6::signals::NeutralModeValue::Coast;
  m_turnMotor.GetConfigurator().Apply( config );
}
