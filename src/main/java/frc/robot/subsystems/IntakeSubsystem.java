// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intakeFeedMotor;
  private TalonSRX shooterFeedMotor;

  private DigitalInput intakeLidar;
  private DigitalInput shooterLidar;

  private double feedSpeed = IntakeConstants.fastFeedRate;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeLidar = Constants.IntakeConstants.intakeLidarSensor;
    shooterLidar = Constants.IntakeConstants.loadedNoteLidarSensor;
    
    intakeFeedMotor = IntakeConstants.intakeFeedMotor;
    intakeFeedMotor.getConfigurator()
        .apply(new MotorOutputConfigs().withInverted(IntakeConstants.intakeFeederInverted));
    shooterFeedMotor = IntakeConstants.shooterFeedMotor;
    shooterFeedMotor.setInverted(IntakeConstants.shooterFeedInverted);
  }

  /** Start arm if enabled and feed the intake */
  public void startAll() { 
    // starts feeders
    intakeFeedMotor.set(feedSpeed);
    shooterFeedMotor.set(TalonSRXControlMode.PercentOutput, feedSpeed);
  }

  public void startIntake() {
    intakeFeedMotor.set(feedSpeed);
  }

  /**
   * Set the speed to run the intake at
   * @param speed speed (out of 1; duty cycle)
   */
  public void setFeedSpeed(double speed) {
    feedSpeed = speed;
  }
  
  public void startShooterFeed() {
    shooterFeedMotor.set(TalonSRXControlMode.PercentOutput, feedSpeed);
  }

  public void stopAll() {
    intakeFeedMotor.stopMotor();
    shooterFeedMotor.neutralOutput();
  }

  public void reverse() {
    intakeFeedMotor.set(-feedSpeed);
    shooterFeedMotor.set(TalonSRXControlMode.PercentOutput, -feedSpeed);
  }

  @Override
  public void periodic() {
  }

  public boolean intakeLidarState()
  {
    return !intakeLidar.get();
  }

  public boolean shooterLidarState()
  {
    return !shooterLidar.get();
  }
}
