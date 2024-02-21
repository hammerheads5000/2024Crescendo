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
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;

public class ShooterHeightPIDSubsystem extends PIDSubsystem {
  TalonFX heightMotor;
  DutyCycleEncoder encoder;

  MutableMeasure<Angle> targetAngle;

  /** Creates a new ShooterHeightPIDSubsystem. */
  public ShooterHeightPIDSubsystem() {
    super(ShooterConstants.heightPID);

    heightMotor = ShooterConstants.heightMotor;
    TalonFXConfigurator config = heightMotor.getConfigurator();
    config.apply(ShooterConstants.heightMotorConfigs);

    encoder = ShooterConstants.heightMotorEncoder;

    targetAngle = ShooterConstants.closeAngle.mutableCopy();

    getController().setTolerance(ShooterConstants.pidDeadband.in(Rotations));
  }

  @Override
  public void useOutput(double output, double setpoint) {
    output += ShooterConstants.arbitraryFeedforward;
    // cap output
    if (Math.abs(output) > ShooterConstants.maxOutput) {
      output = ShooterConstants.maxOutput*Math.signum(output);
    }
    SmartDashboard.putNumber("Motor output", output);
    heightMotor.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    double measured = 0.25+ShooterConstants.encoderValueAt90Deg-encoder.get();
    SmartDashboard.putNumber("Measured Shooter Motor Angle", measured);
    SmartDashboard.putNumber("Measured Shooter Angle", motorPositionToAngle(Rotations.of(measured)).in(Degrees));
    return 0.25 + measured;
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

  public Measure<Angle> motorPositionToAngle(Measure<Angle> motorAngle) {
    double pivX = ShooterConstants.horizontalDistanceToPivot.in(Inches);
    double pivH = ShooterConstants.pivotHeight.in(Inches);
    double l1 = ShooterConstants.topBarLength.in(Inches);
    double l2 = ShooterConstants.bottomBarLength.in(Inches);
    double h = ShooterConstants.motorMountHeight.in(Inches);
    double s = ShooterConstants.motorDistance.in(Inches);
    double m = motorAngle.in(Radians);

    // intermediate calculations
    double b = Math.sqrt(pivX*pivX + pivH*pivH); // distance from shooter hinge to bar pivot
    double c = Math.sqrt(l1*l1 + h*h - 2*l1*h*Math.cos(m)); // distance from bottom of motor mount to air pivot
    double t1 = Math.PI/2 - Math.asin(l1/c * Math.cos(m)); // angle from shooter to air pivot
    double d = Math.sqrt(s*s + c*c - 2*s*c*Math.cos(t1)); // distance from shooter hinge to air pivot
    double t0 = Math.acos((d*d + b*b - l2*l2) / (2*d*b)); // angle from air pivot to hinge to bar pivot
    double t3 = Math.acos((d*d + s*s - c*c) / (2*d*s)); // angle from shooter to air pivot
    
    return Radians.of(t0-t3);
  }
}
