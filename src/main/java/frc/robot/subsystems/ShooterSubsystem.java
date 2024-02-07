// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  TalonFX topMotor;
  TalonFX bottomMotor;

  VelocityTorqueCurrentFOC topRequest;
  VelocityTorqueCurrentFOC bottomRequest;
  public ShooterSubsystem() {}
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(TalonFX topMotor, TalonFX bottomMotor) {
    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;

    topMotor.getConfigurator().apply(ShooterConstants.flywheelGains);
    bottomMotor.getConfigurator().apply(ShooterConstants.flywheelGains);
    
    topRequest = new VelocityTorqueCurrentFOC(ShooterConstants.topSpeed.in(RotationsPerSecond));
    bottomRequest = new VelocityTorqueCurrentFOC(ShooterConstants.topSpeed.in(RotationsPerSecond));
  }

  public void start() {
    topMotor.setControl(topRequest);
    bottomMotor.setControl(bottomRequest);
  }

  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
