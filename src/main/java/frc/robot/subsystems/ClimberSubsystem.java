// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  TalonFX climbMotor;
  DigitalInput lidarSensor;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climbMotor = ClimberConstants.climberMotor;
    climbMotor.getConfigurator().apply(new MotorOutputConfigs()
      .withInverted(ClimberConstants.climberInverted)
      .withNeutralMode(NeutralModeValue.Brake));
    
    lidarSensor = ClimberConstants.limitLidarSensor;
  }

  /**
   * Move the climber
   * @param speed speed to climb at. positive is climbing
   */
  public void climb(double speed) {
    if (reachedClimbLimit() && speed > 0) return;
    climbMotor.set(ClimberConstants.climbSpeed * speed);
  }

  public boolean reachedClimbLimit() {
    return !lidarSensor.get();
  }

  @Override
  public void periodic() {
    if (reachedClimbLimit()) {
      climbMotor.stopMotor();
    }
    
  }
}
