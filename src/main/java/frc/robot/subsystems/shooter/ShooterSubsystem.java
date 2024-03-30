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
import edu.wpi.first.networktables.DoubleSubscriber;
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
  DoubleSubscriber shooterSpeedRequestSubscriber;
  BooleanPublisher shooterAtSpeedPublisher;

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
    shooterSpeedRequestSubscriber = Constants.LoggingConstants.shooterSpeedRequestSubscriber;
    shooterAtSpeedPublisher = Constants.LoggingConstants.shooterAtSpeedPublisher;
  }

  /** Spin wheels up */
  public void spin() {
    Measure<Velocity<Angle>> velocity = RPM.of(shooterSpeedRequestSubscriber.get());
    topMotor.setControl(topRequest.withVelocity(velocity.in(RotationsPerSecond)));
  }

  public void spin(Measure<Velocity<Angle>> speed)
  {
    topMotor.setControl(topRequest.withVelocity(speed.in(RotationsPerSecond)));
  }

  /** Let wheels spin down */
  public void stop() {
    topMotor.stopMotor();
  }

  public Measure<Velocity<Angle>> getFlywheelSpeed() {
    return RotationsPerSecond.of(topMotor.getVelocity().getValueAsDouble());
  }

  public boolean flywheelsAtSpeed() {
    double speedDiff = Math.abs(RPM.of(shooterSpeedRequestSubscriber.get()).in(RotationsPerSecond) - getFlywheelSpeed().in(RotationsPerSecond));
    return speedDiff <= topMotor.getVelocity().getValueAsDouble()*ShooterConstants.readySpeedTolerance;
  }

  @Override
  public void periodic() {
    shooterSpeedPublisher.set(getFlywheelSpeed().in(RPM));
    shooterAtSpeedPublisher.set(flywheelsAtSpeed());
  }
}
