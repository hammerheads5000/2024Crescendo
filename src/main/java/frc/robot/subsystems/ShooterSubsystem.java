// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  TalonFX topMotor;
  TalonFX bottomMotor;

  VelocityTorqueCurrentFOC topRequest;
  VelocityTorqueCurrentFOC bottomRequest;

  DoubleSolenoid solenoid;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    this.topMotor = ShooterConstants.topFlywheel;
    this.bottomMotor = ShooterConstants.bottomFlywheel;
    this.solenoid = ShooterConstants.positioningSoleniod;

    topMotor.getConfigurator().apply(ShooterConstants.flywheelGains);
    bottomMotor.getConfigurator().apply(ShooterConstants.flywheelGains);
    
    topRequest = new VelocityTorqueCurrentFOC(ShooterConstants.topSpeed.in(RotationsPerSecond));
    bottomRequest = new VelocityTorqueCurrentFOC(ShooterConstants.topSpeed.in(RotationsPerSecond));
  }

  public void start() {
    topMotor.setControl(topRequest);
    bottomMotor.setControl(bottomRequest);
  }

  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  public boolean isRaised() {
    return solenoid.get() == Value.kForward;
  }

  public void raise() {
    solenoid.set(Value.kForward);
  }

  public void lower() {
    solenoid.set(Value.kReverse);
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
    // This method will be called once per scheduler run
  }
}
