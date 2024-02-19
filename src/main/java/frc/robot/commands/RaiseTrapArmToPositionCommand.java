// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapHeightPIDSubsystem;

public class RaiseTrapArmToPositionCommand extends Command {
  TrapHeightPIDSubsystem trapHeightPIDSubsystem;
  Measure<Distance> setpoint;

  /** Creates a new RaiseTrapArmToPositionCommand.
   * Waits for arm to reach position
   */
  public RaiseTrapArmToPositionCommand(TrapHeightPIDSubsystem trapHeightPIDSubsystem, Measure<Distance> setpoint) {
    this.trapHeightPIDSubsystem = trapHeightPIDSubsystem;
    this.setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trapHeightPIDSubsystem.setSetpoint(setpoint.in(Inches));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trapHeightPIDSubsystem.getController().atSetpoint();
  }
}
