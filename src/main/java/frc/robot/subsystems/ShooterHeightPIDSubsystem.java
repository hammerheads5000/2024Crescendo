// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;

public class ShooterHeightPIDSubsystem extends PIDSubsystem {
  TalonSRX heightMotor;
  DutyCycleEncoder encoder;
  double maxSpeed;

  MutableMeasure<Angle> targetAngle;

  /** Creates a new ShooterHeightPIDSubsystem. */
  public ShooterHeightPIDSubsystem() {
    super(ShooterConstants.heightPID);

    heightMotor = ShooterConstants.heightMotor;
    heightMotor.setInverted(ShooterConstants.heightMotorInverted);
    heightMotor.setNeutralMode(NeutralMode.Brake);

    encoder = ShooterConstants.heightMotorEncoder;

    maxSpeed = ShooterConstants.maxHeightMotorSpeed.in(RotationsPerSecond) / ShooterConstants.heightMotorGearRatio;

    targetAngle = ShooterConstants.closeAngle.mutableCopy();

    getController().setTolerance(ShooterConstants.pidDeadband.in(Rotations));
  }

  @Override
  public void useOutput(double output, double setpoint) {
    SmartDashboard.putNumber("PID Output", output);
    SmartDashboard.putNumber("Motor output", output / maxSpeed);
    heightMotor.set(TalonSRXControlMode.PercentOutput, output / maxSpeed + ShooterConstants.arbitraryFeedForward);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    SmartDashboard.putNumber("Measured Shooter Motor Angle", 0.25+ShooterConstants.encoderValueAt90Deg-encoder.get());
    return 0.25 + ShooterConstants.encoderValueAt90Deg - encoder.get();
  }

  public void increaseAngle() {
    setTargetAngle(targetAngle.plus(ShooterConstants.manualSpeed));
  }

  public void decreaseAngle() {
    setTargetAngle(targetAngle.minus(ShooterConstants.manualSpeed));
  }

  public void setTargetAngle(Measure<Angle> angle) {
    targetAngle.mut_replace(angle);
    SmartDashboard.putNumber("Shooter Target Angle", targetAngle.in(Degrees));
    SmartDashboard.putNumber("Shooter Motor Target", angleToMotorPosition(targetAngle).in(Degrees));
    setSetpoint(angleToMotorPosition(angle).in(Rotations));
  }

  public Measure<Angle> angleToMotorPosition(Measure<Angle> angle) {
    // shorten variable names for readability
    double pivX = ShooterConstants.horizontalDistanceToPivot.in(Inches);
    double pivH = ShooterConstants.pivotHeight.in(Inches);
    double l1 = ShooterConstants.topBarLength.in(Inches);
    double l2 = ShooterConstants.bottomBarLength.in(Inches);
    double h = ShooterConstants.motorMountHeight.in(Inches);
    double s = ShooterConstants.motorDistance.in(Inches);
    double t = angle.in(Radians);

    // Intermediate calculations
    double b = Math.sqrt(pivX*pivX + pivH*pivH); // distance from shooter hinge to bar pivot
    double t0 = t - Math.atan(pivH / pivX); // shooter from bar pivot
    double l = Math.sqrt(s*s + h*h); // distance from shooter hinge to motor
    // distance from motor to bar pivot
    double c = Math.sqrt(b*b + l*l - 2*b*l* Math.cos(t0 + Math.atan(h/s)));
    double t1 = Math.acos((l1*l1 + c*c - l2*l2) / (2*l1*c)); // top bar angle to bar pivot
    // angle between motor mount and bar pivot
    double t2 = Math.acos(h/l) - Math.acos((c*c + l*l - b*b) / (2*c*l));

    return Radians.of(t1 + t2);
  }
}
