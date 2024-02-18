// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumSet;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private TalonSRX armRaiseMotor;
  private TalonSRX armFeedMotor;
  private TalonFX intakeFeedMotor;
  private TalonSRX shooterFeedMotor;

  private boolean armEnabled = false;
  private EventLoop eventLoop = new EventLoop();
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    armRaiseMotor = IntakeConstants.armRaiseMotor;
    armFeedMotor = IntakeConstants.armFeedMotor;

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

  public void startAll() {
    if (armEnabled){
      armRaiseMotor.set(TalonSRXControlMode.PercentOutput, -IntakeConstants.armDropDutyCycle); // pushes arm down
      armFeedMotor.set(TalonSRXControlMode.PercentOutput, IntakeConstants.feederDutyCycle);
    }
    
    // starts feeders
    intakeFeedMotor.set(IntakeConstants.feederDutyCycle);
    shooterFeedMotor.set(TalonSRXControlMode.PercentOutput, IntakeConstants.feederDutyCycle);
  }

  public void raiseArm() {
    if (armEnabled) {
      armRaiseMotor.set(TalonSRXControlMode.PercentOutput, IntakeConstants.armRaiseDutyCycle); // raises arm
      armFeedMotor.neutralOutput();;
    }
  }

  public void startShooterFeed() {
    shooterFeedMotor.set(TalonSRXControlMode.PercentOutput, IntakeConstants.feederDutyCycle);
  }

  public void stopFeeding() {
    intakeFeedMotor.stopMotor();
    shooterFeedMotor.neutralOutput();
  }

  public void reverse() {
    intakeFeedMotor.set(-IntakeConstants.feederDutyCycle);
    shooterFeedMotor.set(TalonSRXControlMode.PercentOutput, -IntakeConstants.feederDutyCycle);
  }

  public void stopArm() {
    armRaiseMotor.neutralOutput();

  }

  public void setArmEnabled(boolean armEnabled) {
    this.armEnabled = armEnabled;
  }

  @Override
  public void periodic() {
    eventLoop.poll(); // updates eventLoop
  }
}
