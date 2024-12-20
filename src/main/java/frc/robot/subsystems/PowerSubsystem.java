// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PowerSubsystem extends SubsystemBase {
  PowerDistribution pdh = Constants.pdh;
  /** Creates a new PowerSubsystem. */
  public PowerSubsystem() {
    SmartDashboard.putData(pdh);
  }

  public double getVoltage() {
    return pdh.getVoltage();
  }

  public double climbCurrent() {
    return pdh.getCurrent(14);
  }

  @Override
  public void periodic() {
  }
}
