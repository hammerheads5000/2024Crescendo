// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumSet;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private TalonSRX armRaiseMotor;
  private TalonSRX armFeedMotor;
  private TalonFX intakeFeedMotor;
  private TalonSRX shooterFeedMotor;
  private boolean IntakeON = false;
  private boolean feederON = false;
  private boolean IntakeFAST = true;
  private boolean armEnabled = false;

  private DigitalInput intakeLidar;
  private DigitalInput shooterLidar;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    armRaiseMotor = IntakeConstants.armRaiseMotor;
    armFeedMotor = IntakeConstants.armFeedMotor;

    intakeLidar = Constants.IntakeConstants.intakeLidarSensor;
    shooterLidar = Constants.IntakeConstants.loadedNoteLidarSensor;
    
    intakeFeedMotor = IntakeConstants.intakeFeedMotor;
    intakeFeedMotor.getConfigurator()
        .apply(new MotorOutputConfigs().withInverted(IntakeConstants.intakeFeederInverted));
    shooterFeedMotor = IntakeConstants.shooterFeedMotor;
    shooterFeedMotor.setInverted(IntakeConstants.shooterFeedInverted);

    SmartDashboard.putBoolean("Drop-Down Intake Enabled", true);

    // allows SmartDashboard control of whether to enable arm
    NetworkTableListener.createListener(
      Constants.inst.getTopic("/SmartDashboard/Drop-Down Intake Enabled"),
      EnumSet.of(NetworkTableEvent.Kind.kValueAll), // listens for any value change
        event -> {
          setArmEnabled(event.valueData.value.getBoolean());
        });
  }

  /** Start arm if enabled and feed the intake */
  public void startAll() { 
    // starts feeders
    intakeFeedMotor.set(IntakeConstants.fastFeedRate);
    IntakeON = true;
    IntakeFAST = true;
  }
  public void startFeeder()
  {
    intakeFeedMotor.set(IntakeConstants.fastFeedRate);
  }

  /**
   * Set the speed to run the intake at
   * @param speed speed (out of 1; duty cycle)
   */
  public void setIntakeSpeed(double speed) {
    intakeFeedMotor.set(speed);
  }
  
  public void raiseArm() {
    if (armEnabled) {
      armRaiseMotor.set(TalonSRXControlMode.PercentOutput, IntakeConstants.armRaiseDutyCycle); // raises arm
      armFeedMotor.neutralOutput();
    }
  }

  public void startShooterFeed() {
    shooterFeedMotor.set(TalonSRXControlMode.PercentOutput, IntakeConstants.fastFeedRate);
    System.out.println("shoot");
  }

  public void stopFeeding() {
    intakeFeedMotor.stopMotor();
    shooterFeedMotor.neutralOutput();
    IntakeON = false;
  }

  public void reverse() {
    intakeFeedMotor.set(-IntakeConstants.fastFeedRate);
    shooterFeedMotor.set(TalonSRXControlMode.PercentOutput, -IntakeConstants.fastFeedRate);
  }

  public void stopArm() {
    armRaiseMotor.neutralOutput();
  }

  public void setArmEnabled(boolean armEnabled) {
    this.armEnabled = armEnabled;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("IntakeLIDAR", intakeLidarState());
    if(IntakeON)
    {
      if(IntakeFAST && intakeLidarState())
      {
        SmartDashboard.putBoolean("YAY", true);
        setIntakeSpeed(IntakeConstants.slowFeedRate);
        IntakeFAST = false;
        intakeFeedMotor.set(IntakeConstants.fastFeedRate);
      }
      if (IntakeON && shooterLidarState())
      {
        stopFeeding();
        IntakeON = false;
      }
    }
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
