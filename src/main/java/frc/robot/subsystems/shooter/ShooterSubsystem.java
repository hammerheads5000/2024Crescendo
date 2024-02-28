// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
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

  /** Spin wheels up */
  public void start() {
    topMotor.setControl(topRequest);
  }

  /** Let wheels spin down */
  public void stop() {
    topMotor.stopMotor();
  }

  public Measure<Velocity<Angle>> getFlywheelSpeed() {
    return RotationsPerSecond.of(topMotor.getVelocity().getValueAsDouble());
  }

  public boolean flywheelsAtSpeed() {
    return getFlywheelSpeed().gte(ShooterConstants.topSpeed.minus(ShooterConstants.readySpeedTolerance));
  }

  public boolean flywheelsAtCloseSpeed() {
    return getFlywheelSpeed().gte(ShooterConstants.topSpeed.minus(ShooterConstants.closeSpeedTolerance));
  }

  @Override
  public void periodic() {
  }
}
