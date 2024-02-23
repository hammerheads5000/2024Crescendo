// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  TalonFX topMotor;
  TalonFX bottomMotor;

  VelocityTorqueCurrentFOC topRequest;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    this.topMotor = ShooterConstants.topFlywheel;
    this.bottomMotor = ShooterConstants.bottomFlywheel;
    
    topMotor.getConfigurator().apply(ShooterConstants.flywheelGains);
    topMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(ShooterConstants.topFlywheelInverted));
    bottomMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(ShooterConstants.bottomFlywheelInverted));

    topRequest = new VelocityTorqueCurrentFOC(ShooterConstants.topSpeed.in(RotationsPerSecond));
    bottomMotor.setControl(new StrictFollower(topMotor.getDeviceID()));
  }

  public void start() {
    topMotor.setControl(topRequest);
  }

  public void stop() {
    topMotor.stopMotor();
  }

  @Override
  public void periodic() {
  }
}
