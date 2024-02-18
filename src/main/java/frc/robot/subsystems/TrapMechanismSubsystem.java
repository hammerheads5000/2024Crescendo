// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrapConstants;

public class TrapMechanismSubsystem extends SubsystemBase {
  TalonSRX rollerMotor;

  Servo linearActuator;

  /** Creates a new TrapMechanismSubsystem. */
  public TrapMechanismSubsystem() {
    heightControlMotor = TrapConstants.heightControlMotor;
    heightControlMotor.setInverted(TrapConstants.heightMotorInverted);
    rollerMotor = TrapConstants.rollerMotor;
    rollerMotor.setInverted(TrapConstants.rollerInverted);

    linearActuator = TrapConstants.linearActuator;    
    linearActuator.setBoundsMicroseconds(TrapConstants.maxMicroseconds, 0, 
                                        TrapConstants.centerMicroseconds, 0, 
                                        TrapConstants.minMicroseconds);

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

  public void raise() {
    heightControlMotor.set(TrapConstants.raiseSpeed);
  }

  public void lower() {
    heightControlMotor.set(-TrapConstants.lowerSpeed);
  }

  public void stopHeight() {
    heightControlMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
