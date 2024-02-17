// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrapConstants;

public class TrapMechanismSubsystem extends SubsystemBase {
  TalonSRX rollerMotor;

  Servo linearActuator;

  DigitalInput noteSensor;

  /** Creates a new TrapMechanismSubsystem. */
  public TrapMechanismSubsystem() {
    rollerMotor = TrapConstants.rollerMotor;

    linearActuator = TrapConstants.linearActuator;    
    linearActuator.setBoundsMicroseconds(TrapConstants.maxMicroseconds, 0, 
                                        TrapConstants.centerMicroseconds, 0, 
                                        TrapConstants.minMicroseconds);

    noteSensor = TrapConstants.noteSensor;
  }

  public boolean isNoteDetected() {
    return noteSensor.get();
  }

  public void intake() {
    rollerMotor.set(TalonSRXControlMode.PercentOutput, TrapConstants.intakeSpeed);
  }

  public void expel() {
    rollerMotor.set(TalonSRXControlMode.PercentOutput, -TrapConstants.expelSpeed);
  }

  public void stopRollers() {
    rollerMotor.neutralOutput();
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
