// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.trapmechanism;

import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.TrapConstants;

public class TrapHeightPIDSubsystem extends PIDSubsystem {
  CANSparkMax heightControlMotor;
  Encoder encoder;

  ColorSensorV3 colorSensor;
  ColorMatch colorMatch;

  DoublePublisher trapHeightPublisher = Constants.LoggingConstants.trapHeightPublisher;
  BooleanPublisher colorSensorPublisher = Constants.LoggingConstants.colorSensorPublisher;
  DoublePublisher trapMechanismSpeedPublisher = Constants.LoggingConstants.trapMechanismSpeedPublisher;
  DoublePublisher trapMechanismSetpointPublisher = Constants.LoggingConstants.trapMechanismSetpointPublisher;
  
  /** Creates a new TrapHeightPIDSubsystem. */
  public TrapHeightPIDSubsystem() {
    super(TrapConstants.heightPIDController);

    heightControlMotor = TrapConstants.heightControlMotor;
    heightControlMotor.setInverted(TrapConstants.heightMotorInverted);

    colorSensor = TrapConstants.colorSensor;
    colorMatch = new ColorMatch();
    colorMatch.addColorMatch(TrapConstants.colorToMatch);

    encoder = TrapConstants.heightEncoder;
    encoder.setDistancePerPulse(TrapConstants.distancePerPulse.in(Inches));
    resetEncoder();

    getController().setTolerance(TrapConstants.heightTolerance.in(Inches));
  }

  public void useOutput(double output, double setpoint) {
    double dutyCycle = output*TrapConstants.raiseSpeed;
    dutyCycle = Math.max(Math.min(dutyCycle, 1.0), -1.0); // cap between [-1,1]
    heightControlMotor.set(dutyCycle);

    trapMechanismSetpointPublisher.set(setpoint);
    trapMechanismSpeedPublisher.set(dutyCycle);
  }

  @Override
  public double getMeasurement() {
    trapHeightPublisher.set(encoder.getDistance());
    colorSensorPublisher.set(colorDetected());
    
    return encoder.getDistance();
  }

  public void raise(double speed) {
    heightControlMotor.set(TrapConstants.raiseSpeed * speed);
  }

  public void lower() {
    heightControlMotor.set(-TrapConstants.lowerSpeed);
  }

  /** Stop the height control motor */
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
    setSetpoint(TrapConstants.homePosition.in(Inches));
  }

  public void moveToTrap() {
    setSetpoint(TrapConstants.trapPosition.in(Inches));
  }

  public void moveToSource() {
    setSetpoint(TrapConstants.sourcePosition.in(Inches));
  }

  public boolean colorDetected() {
    ColorMatchResult result = colorMatch.matchColor(colorSensor.getColor());
    return result != null;
  }
}
