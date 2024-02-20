// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbCommand extends Command {
  /** Creates a new ClimbCommand. */
  ClimberSubsystem climb_Sub;
  CommandXboxController controller;
  public ClimbCommand(ClimberSubsystem climb_Sub, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controller = controller;
    this.climb_Sub = climb_Sub;
    addRequirements(climb_Sub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(Math.abs(controller.getLeftY()) > Constants.SwerveConstants.controllerDeadband)
    {
      climb_Sub.Climb(controller.getLeftY());
    }
    else {
      climb_Sub.Climb(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
