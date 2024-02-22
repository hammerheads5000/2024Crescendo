// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapHeightPIDSubsystem;

public class HomeTrapArmCommand extends Command {
  TrapHeightPIDSubsystem trapHeightPIDSubsystem;
  /** Creates a new HomeTrapArmCommand. */
  public HomeTrapArmCommand(TrapHeightPIDSubsystem trapHeightPIDSubsystem) {
    this.trapHeightPIDSubsystem = trapHeightPIDSubsystem;
    addRequirements(trapHeightPIDSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (isFinished()) return;
    trapHeightPIDSubsystem.disable();
    trapHeightPIDSubsystem.lower();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trapHeightPIDSubsystem.stop();
    if (!interrupted) {
      trapHeightPIDSubsystem.resetEncoder();
      trapHeightPIDSubsystem.moveToHome();
    }
    else {
      trapHeightPIDSubsystem.setSetpoint(trapHeightPIDSubsystem.getMeasurement());
    }
    trapHeightPIDSubsystem.enable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trapHeightPIDSubsystem.colorDetected();
  }
}
