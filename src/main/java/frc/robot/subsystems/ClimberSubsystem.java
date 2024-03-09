// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  TalonFX climbMotor;
  DigitalInput lidarSensor;
  BooleanPublisher climberDownPublisher = Constants.LoggingConstants.climberDownPublisher;
  DoublePublisher climberSpeedPublisher = Constants.LoggingConstants.climberSpeedPublisher;
  LightsSubsystem lightsSubsystem;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(LightsSubsystem lightsSubsystem) {
    climbMotor = ClimberConstants.climberMotor;
    climbMotor.getConfigurator().apply(new MotorOutputConfigs()
      .withInverted(ClimberConstants.climberInverted)
      .withNeutralMode(NeutralModeValue.Brake));
    this.lightsSubsystem = lightsSubsystem;
    lidarSensor = ClimberConstants.limitLidarSensor;
  }

  /**
   * Move the climber
   * @param speed speed to climb at. positive is climbing
   */
  public void climb(double speed) {
    //if (reachedClimbLimit() && speed > 0) return;
    climbMotor.set(ClimberConstants.climbSpeed * speed);
    climberSpeedPublisher.set(ClimberConstants.climbSpeed * speed);
  }

  public boolean reachedClimbLimit() {
    return !lidarSensor.get();
  }

  public void stopMotor()
  {
    climbMotor.stopMotor();
  }
  
  @Override
  public void periodic() {
    climberDownPublisher.set(reachedClimbLimit());
    if (reachedClimbLimit()) {
      climbMotor.stopMotor();
    }
  }
}
