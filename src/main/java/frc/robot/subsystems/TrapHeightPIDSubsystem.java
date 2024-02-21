// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.TrapConstants;

public class TrapHeightPIDSubsystem extends PIDSubsystem {
  CANSparkMax heightControlMotor;

  DigitalInput homeLimitSwitch;
  Encoder encoder;
  
  /** Creates a new TrapHeightPIDSubsystem. */
  public TrapHeightPIDSubsystem() {
    super(TrapConstants.heightPIDController);

    heightControlMotor = TrapConstants.heightControlMotor;
    heightControlMotor.setInverted(TrapConstants.heightMotorInverted);
    homeLimitSwitch = TrapConstants.homeLimitSwitch;
    encoder = TrapConstants.heightEncoder;
    encoder.setDistancePerPulse(TrapConstants.distancePerPulse.in(Inches));

    getController().setTolerance(TrapConstants.heightTolerance.in(Inches));
  }

  @Override
  public void useOutput(double output, double setpoint) {
    heightControlMotor.set(TrapConstants.raiseSpeed*output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    if (homeLimitSwitch.get()) {
      encoder.reset();
      return 0;
    }
    return encoder.getDistance();
  }

  public void raise(double speed) {
    heightControlMotor.set(TrapConstants.raiseSpeed * speed);
  }

  public void lower() {
    heightControlMotor.set(-TrapConstants.lowerSpeed);
  }

  public void stop() {
    heightControlMotor.stopMotor();
  }

  public void resetEncoder() {
    encoder.reset();
  }

  public void moveToAmp() {
    setSetpoint(TrapConstants.ampPosition.in(Inches));
  }

  public void moveToHome() {
    setSetpoint(0);
  }

  public void moveToTrap() {
    setSetpoint(TrapConstants.trapPosition.in(Inches));
  }

  public void moveToSource() {
    setSetpoint(TrapConstants.sourcePosition.in(Inches));
  }

  public boolean getLimitSwitch() {
    return homeLimitSwitch.get();
  }
}
