// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class FaceSpeaker extends Command {
  Swerve swerve;
  CommandXboxController controller;
  Translation3d speakerPos;

  /** Creates a new TeleopSwerve. */
  public FaceSpeaker(Swerve swerve, CommandXboxController controller) {
    this.swerve = swerve;
    this.controller = controller;

    Optional<Alliance> team = DriverStation.getAlliance();
    if (team.isPresent() && team.get() == Alliance.Red){
      speakerPos = SwerveConstants.redSpeakerPos;
    }else{ //auto blue if there is no team because why not
      speakerPos = SwerveConstants.blueSpeakerPos;
    }
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d targetAngle = new Rotation2d(swerve.getPose().getX() - speakerPos.getX(), swerve.getPose().getY() - speakerPos.getY());
    swerve.driveFacingAngle(
        SwerveConstants.maxDriveSpeed
            .times(Math.abs(controller.getLeftY()) >= SwerveConstants.controllerDeadband ? controller.getLeftY() : 0),
        SwerveConstants.maxDriveSpeed
            .times(Math.abs(controller.getLeftX()) >= SwerveConstants.controllerDeadband ? -controller.getLeftX() : 0),
        targetAngle);
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
