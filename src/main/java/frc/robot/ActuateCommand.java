// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ActuatorSubsystem;

public class ActuateCommand extends Command {
  ActuatorSubsystem actuatorSubsystem;
  /** Creates a new ActuateCommand. */
  public ActuateCommand(ActuatorSubsystem actuatorSubsystem) {
    this.actuatorSubsystem = actuatorSubsystem;
    addRequirements(actuatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    actuatorSubsystem.extend();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    actuatorSubsystem.contract();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
