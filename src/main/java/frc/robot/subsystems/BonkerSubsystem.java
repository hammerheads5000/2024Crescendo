// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TrapConstants;

public class BonkerSubsystem extends SubsystemBase {
  /** Creates a new BonkerSubsystem. */
  Servo bonker;
  public BonkerSubsystem() {
    bonker = Constants.TrapConstants.bonker;
    bonker.setBoundsMicroseconds(
      TrapConstants.maxMicrosecondsBonk, 
      TrapConstants.deadbandBonk,
      TrapConstants.centerMicrosecondsBonk,
      TrapConstants.deadbandBonk, 
      TrapConstants.minMicrosecondsBonk);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Bonker angle", bonker.getAngle());
  }

  public void bonkForward()
  {
   bonker.set(0);
  }

  public void bonkBackward()
  {
    bonker.set(1);
  }

  public void toggleBonk()
  {
    if(bonker.get() > .1)
    {
      bonker.set(0);
    }
    else{
      bonker.set(1);
    }
  }
}
