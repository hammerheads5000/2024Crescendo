// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.trapmechanism;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TrapConstants;

public class TrapMechanismSubsystem extends SubsystemBase {
  TalonSRX rollerMotor;
  DigitalInput lidarSensor;
  BooleanPublisher ActuatorPublisher = Constants.LoggingConstants.ActuatorPublisher;
  BooleanPublisher TrapLIDARPublisher = Constants.LoggingConstants.TrapLIDARPublisher;
  DoublePublisher trapMechanismDoublePublisher = Constants.LoggingConstants.TrapMechanismSpeedPublisher;
  Servo linearActuator;

  /** Creates a new TrapMechanismSubsystem. */
  public TrapMechanismSubsystem() {
    rollerMotor = TrapConstants.rollerMotor;
    rollerMotor.setInverted(TrapConstants.rollerInverted);

    linearActuator = TrapConstants.linearActuator;    
    linearActuator.setBoundsMicroseconds(TrapConstants.maxMicroseconds, 0, 
                                        TrapConstants.centerMicroseconds, 0, 
                                        TrapConstants.minMicroseconds);
                            
    lidarSensor = TrapConstants.noteDetectionLidarSensor;
  }


  public void forward() {
    rollerMotor.set(TalonSRXControlMode.PercentOutput, -TrapConstants.intakeSpeed);
    trapMechanismDoublePublisher.set(-TrapConstants.intakeSpeed);
  }

  public void reverse() {
    rollerMotor.set(TalonSRXControlMode.PercentOutput, TrapConstants.expelSpeed);
    trapMechanismDoublePublisher.set(TrapConstants.intakeSpeed);
  }

  public void stopRollers() {
    rollerMotor.neutralOutput();
    trapMechanismDoublePublisher.set(0);
  }

  public double getActuator(){
    return linearActuator.get();
  }

  public void extendActuator(){
    linearActuator.set(1.0);
    ActuatorPublisher.set(true);
  }

  public void contractActuator(){
    linearActuator.set(0.0);
    ActuatorPublisher.set(false);
  }

  public void toggleActuator() {
    if (linearActuator.get() == 1.0) {
      contractActuator();
      ActuatorPublisher.set(false);
      return;
    }
    extendActuator();
    ActuatorPublisher.set(true);
  }

  /** Set actuator position for shooting into amp */
  public void moveActuatorForAmp() {
    linearActuator.set(Constants.TrapConstants.ampActuatorPosition);
  }

  public boolean isNoteDetected() {
    return !lidarSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    TrapLIDARPublisher.set(isNoteDetected());
  }
}
