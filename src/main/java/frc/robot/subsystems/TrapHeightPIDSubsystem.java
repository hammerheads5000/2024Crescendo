// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.TrapConstants;

public class TrapHeightPIDSubsystem extends PIDSubsystem {
  TalonFX heightControlMotor;

  DigitalInput homeLimitSwitch;
  Encoder encoder;
  
  /** Creates a new TrapHeightPIDSubsystem. */
  public TrapHeightPIDSubsystem() {
    super(TrapConstants.heightPIDController);

    heightControlMotor = TrapConstants.heightControlMotor;
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

  public void lower() {
    heightControlMotor.set(-TrapConstants.lowerSpeed);
  }

  public void stop() {
    heightControlMotor.stopMotor();
  }

  public void resetEncoder() {
    encoder.reset();
  }

  public boolean getLimitSwitch() {
    return homeLimitSwitch.get();
  }
}
