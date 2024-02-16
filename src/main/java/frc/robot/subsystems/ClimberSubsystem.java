// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  TalonFX climbMotor;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climbMotor = ClimberConstants.climberMotor;
  }

  public void climbUp() {
    climbMotor.set(ClimberConstants.climbSpeed);
  }

  public void climbDown() {
    climbMotor.set(-ClimberConstants.climbSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
