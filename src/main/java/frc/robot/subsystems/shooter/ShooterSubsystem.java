// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  TalonFX topMotor;
  TalonFX bottomMotor;
  VelocityTorqueCurrentFOC topRequest;
  
  DoublePublisher shooterSpeedPublisher;
  DoublePublisher shooterSpeedRequestPublisher;
  BooleanPublisher shooterAtSpeedPublisher;
  BooleanPublisher shooterNearSpeedPublisher;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    this.topMotor = ShooterConstants.topFlywheel;
    this.bottomMotor = ShooterConstants.bottomFlywheel;
    
    topMotor.getConfigurator().apply(ShooterConstants.flywheelGains);
    topMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(ShooterConstants.topFlywheelInverted));
    bottomMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(ShooterConstants.bottomFlywheelInverted));

    topRequest = new VelocityTorqueCurrentFOC(ShooterConstants.topSpeed.in(RotationsPerSecond));
    topRequest.Acceleration = ShooterConstants.flywheelAccel.in(RotationsPerSecond.per(Second));
    bottomMotor.setControl(new StrictFollower(topMotor.getDeviceID()));

    shooterSpeedPublisher = Constants.LoggingConstants.shooterSpeedPublisher;
    shooterSpeedRequestPublisher = Constants.LoggingConstants.shooterSpeedRequestPublisher;
    shooterAtSpeedPublisher = Constants.LoggingConstants.shooterAtSpeedPublisher;
    shooterNearSpeedPublisher = Constants.LoggingConstants.shooterNearSpeedPublisher;
  }

  /** Spin wheels up */
  public void start() {
    topMotor.setControl(topRequest);
    shooterSpeedRequestPublisher.set(RotationsPerSecond.of(topRequest.Velocity).in(RPM));
  }

  /** Let wheels spin down */
  public void stop() {
    topMotor.stopMotor();
    shooterSpeedRequestPublisher.set(0);
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
    if(flywheelsAtSpeed())
    {

    }
    else if(flywheelsAtCloseSpeed())
    {
      
    }
    shooterSpeedPublisher.set(getFlywheelSpeed().in(RPM));
    shooterAtSpeedPublisher.set(flywheelsAtSpeed());
    shooterNearSpeedPublisher.set(flywheelsAtCloseSpeed());
  }
}
