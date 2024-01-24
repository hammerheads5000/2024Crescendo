// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends Command {
  Swerve swerve;
  CommandXboxController controller;

  /** Creates a new TeleopSwerve. */
  public TeleopSwerve(Swerve swerve, CommandXboxController controller) {
    this.swerve = swerve;
    this.controller = controller;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(SwerveConstants.maxDriveSpeed.times(controller.getLeftX()), 
                SwerveConstants.maxDriveSpeed.times(controller.getLeftY()), 
                SwerveConstants.maxRotSpeed.times(-controller.getRightX())
    );
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