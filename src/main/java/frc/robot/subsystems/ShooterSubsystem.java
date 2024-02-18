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
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  TalonFX topMotor;
  TalonFX bottomMotor;

  VelocityTorqueCurrentFOC topRequest;
  VelocityTorqueCurrentFOC bottomRequest;

  DoubleSolenoid solenoid;

  TalonSRX heightMotor;
  DutyCycleEncoder encoder;

  MutableMeasure<Angle> targetAngle;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    this.topMotor = ShooterConstants.topFlywheel;
    this.bottomMotor = ShooterConstants.bottomFlywheel;
    
    topMotor.getConfigurator().apply(ShooterConstants.flywheelGains);
    topMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(ShooterConstants.topFlywheelInverted));
    bottomMotor.getConfigurator().apply(ShooterConstants.flywheelGains);
    bottomMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(ShooterConstants.bottomFlywheelInverted));

    topRequest = new VelocityTorqueCurrentFOC(ShooterConstants.topSpeed.in(RotationsPerSecond));
    bottomRequest = new VelocityTorqueCurrentFOC(ShooterConstants.topSpeed.in(RotationsPerSecond));

    this.heightMotor = ShooterConstants.heightMotor;
    heightMotor.setInverted(ShooterConstants.heightMotorInverted);
    this.encoder = ShooterConstants.heightMotorEncoder;

    targetAngle = ShooterConstants.closeAngle.plus(ShooterConstants.farAngle).divide(2).mutableCopy();

    SlotConfiguration gains = new SlotConfiguration();
    gains.kF = ShooterConstants.kF;
    gains.kP = ShooterConstants.kP;
    gains.kI = ShooterConstants.kI;
    gains.kD = ShooterConstants.kD;
    heightMotor.configureSlot(gains);
    heightMotor.configMotionAcceleration(ShooterConstants.motionMagicAccel);
    heightMotor.configMotionCruiseVelocity(ShooterConstants.motionMagicVel);
    heightMotor.configMotionSCurveStrength(ShooterConstants.motionMagicSCurve);
    heightMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void start() {
    topMotor.setControl(topRequest);
    bottomMotor.setControl(bottomRequest);
  }

  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  public void increaseAngle() {
    targetAngle.mut_plus(ShooterConstants.manualSpeed);
    if (targetAngle.gt(ShooterConstants.closeAngle)) {
      targetAngle.mut_replace(ShooterConstants.closeAngle);
    }
    updateTargetAngle();
  }

  public void decreaseAngle() {
    targetAngle.mut_minus(ShooterConstants.manualSpeed);
    if (targetAngle.lt(ShooterConstants.farAngle)) {
      targetAngle.mut_replace(ShooterConstants.farAngle);
    }
    updateTargetAngle();
  }

  public void setTargetAngle(Measure<Angle> angle) {
    targetAngle.mut_replace(angle);
    updateTargetAngle();
  }
  
  public void updateTargetAngle() {
    Measure<Angle> motorAngle = angleToMotorPosition(targetAngle);
    SmartDashboard.putNumber("Target angle deg", targetAngle.in(Degrees));
    double encoderOutput = angleToEncoderRelative(motorAngle);
    SmartDashboard.putNumber("Desired motor encoder", encoderOutput);
    SmartDashboard.putNumber("Current motor encoder", heightMotor.getSelectedSensorPosition());
    heightMotor.set(TalonSRXControlMode.Position, encoderOutput);
    SmartDashboard.putNumber("Height motor out", heightMotor.getMotorOutputPercent());
  }

  public Measure<Angle> getEncoderAngle() {
    double raw = encoder.getAbsolutePosition();
    // 0.25 is because offset is measured at 90 degrees
    Measure<Angle> actualAngle = Rotations.of(0.25 + ShooterConstants.encoderValueAt90Deg - raw); 
    return actualAngle;
  }

  private double angleToEncoderRelative(Measure<Angle> angle) {
    double currentEncoderRelative = heightMotor.getSelectedSensorPosition();
    Measure<Angle> currentAbsolutePosToAngle = angle.minus(Rotations.of(encoder.getAbsolutePosition()));
    double angleDiffEncoderUnits = currentAbsolutePosToAngle.in(Rotations) * ShooterConstants.sensorUnitsPerRotation;

    return currentEncoderRelative + angleDiffEncoderUnits;
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Raw Shooter Lift Encoder", encoder.get());
  }
}
