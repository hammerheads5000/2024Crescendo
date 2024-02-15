// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumSet;

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
  private MotorController armRaiseMotor;
  private MotorController armFeedMotor;
  private MotorController intakeFeedMotor;
  private MotorController shooterFeedMotor;

  private boolean armEnabled = false;
  private EventLoop eventLoop = new EventLoop();
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    armRaiseMotor = IntakeConstants.armRaiseMotor;
    armFeedMotor = IntakeConstants.armFeedMotor;
    intakeFeedMotor = IntakeConstants.intakeFeedMotor;
    shooterFeedMotor = IntakeConstants.shooterFeedMotor;

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
      armRaiseMotor.set(-IntakeConstants.armDropDutyCycle); // pushes arm down
      armFeedMotor.set(IntakeConstants.feederDutyCycle);
    }
    
    // starts feeders
    intakeFeedMotor.set(IntakeConstants.feederDutyCycle);
    shooterFeedMotor.set(IntakeConstants.feederDutyCycle);
  }

  public void raiseArm() {
    if (armEnabled) {
      armRaiseMotor.set(IntakeConstants.armRaiseDutyCycle); // raises arm
      armFeedMotor.stopMotor();
    }
  }

  public void stopFeeding() {
    intakeFeedMotor.stopMotor();
    shooterFeedMotor.stopMotor();
  }

  public void stopArm() {
    armRaiseMotor.stopMotor();

  }

  public void setArmEnabled(boolean armEnabled) {
    this.armEnabled = armEnabled;
  }

  @Override
  public void periodic() {
    eventLoop.poll(); // updates eventLoop
  }
}
