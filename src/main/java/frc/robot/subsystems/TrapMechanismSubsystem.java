// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrapConstants;

public class TrapMechanismSubsystem extends SubsystemBase {
  TalonFX heightControlMotor;
  TalonFX rollerMotor;

  Servo linearActuator;

  DigitalInput ampLimitSwitch;
  DigitalInput trapLimitSwitch;
  DigitalInput homeLimitSwitch;

  DigitalInput noteSensor;

  /** Creates a new TrapMechanismSubsystem. */
  public TrapMechanismSubsystem() {
    heightControlMotor = TrapConstants.heightControlMotor;
    rollerMotor = TrapConstants.rollerMotor;

    linearActuator = TrapConstants.linearActuator;    
    linearActuator.setBoundsMicroseconds(2000, 0, 1500, 0, 1000);

    ampLimitSwitch = TrapConstants.ampLimitSwitch;
    trapLimitSwitch = TrapConstants.trapLimitSwitch;
    homeLimitSwitch = TrapConstants.homeLimitSwitch;

    noteSensor = TrapConstants.noteSensor;
  }

  public void raise() {
    heightControlMotor.set(TrapConstants.raiseSpeed);
  }

  public void lower() {
    heightControlMotor.set(-TrapConstants.raiseSpeed);
  }

  public void stopHeight() {
    heightControlMotor.stopMotor();
  }

  public boolean isNoteDetected() {
    return noteSensor.get();
  }

  public boolean isLowered() {
    return homeLimitSwitch.get();
  }

  public boolean isAtAmp() {
    return ampLimitSwitch.get();
  }

  public boolean isAtTrap() {
    return trapLimitSwitch.get();
  }

  public void intake() {
    rollerMotor.set(TrapConstants.intakeSpeed);
  }

  public void expel() {
    rollerMotor.set(-TrapConstants.expelSpeed);
  }

  public void stopRollers() {
    rollerMotor.stopMotor();
  }

public double getActuator(){
  return linearActuator.get();
}

  public void extendActuator(){
    linearActuator.set(1.0);
  }
  public void contractActuator(){
    linearActuator.set(0.0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
