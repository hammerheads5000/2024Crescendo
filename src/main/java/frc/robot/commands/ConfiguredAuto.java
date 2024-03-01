// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.GoalPoint;
import frc.robot.commands.intake.IntakeCommandGroup;
import frc.robot.subsystems.Swerve;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ConfiguredAuto extends Command {
  /** Creates a new ConfiguredAuto. */
  ArrayList<GoalPoint> goalPointsList = Constants.AutoConstants.AutoGoalPoints;
  GoalPoint startGoalPoint;
  GoalPoint endGoalPoint;
  boolean isFinished = false;
  int startIndex;
  int nextIndex;
  Swerve swerve;
  Command intakeCommand;
  AimShooterCommand shootCommand;
  public ConfiguredAuto(Swerve swerve, int startIndex, int nextIndex,Command intakeCommand, AimShooterCommand shootCommand) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.startIndex = startIndex;
    this.nextIndex = nextIndex;
    this.intakeCommand = intakeCommand;
    this.shootCommand = shootCommand;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    startGoalPoint = goalPointsList.get(startIndex);
    endGoalPoint = goalPointsList.get(nextIndex);
    SmartDashboard.putBoolean("isFinished", isFinished);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(endGoalPoint.getWillIntake())
    {
      double deltaX = startGoalPoint.getX() - endGoalPoint.getX();
      double deltaY = startGoalPoint.getY() - endGoalPoint.getY();
      double ScaleConstant = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
      deltaX *= ScaleConstant;
      deltaY *= ScaleConstant;
      double X = deltaX + startGoalPoint.getX();
      double Y = deltaY + startGoalPoint.getY();
      GoalPoint ProximityPoint = new GoalPoint(X, Y, endGoalPoint.GetWillShoot(), endGoalPoint.getWillIntake(), 0);
      Command PathFollowCommand = swerve.CreatePathFollowCommand(startGoalPoint, ProximityPoint);
      PathFollowCommand.asProxy();
      System.out.println("yay?");
      //intakeCommand.asProxy();
    }
    else
    {
      Command PathFollowCommand = swerve.CreatePathFollowCommand(startGoalPoint, endGoalPoint);
      PathFollowCommand.asProxy();
      SmartDashboard.putNumber("Requested X", endGoalPoint.getX());
      SmartDashboard.putNumber("actual X", swerve.getPose().getX());
    }
    if(endGoalPoint.GetWillShoot())
    {
     // shootCommand.asProxy(); // will only aim shooter I beleive, implement fully auto command when done
    }
    if(nextIndex < goalPointsList.size())
    {
      new ConfiguredAuto(swerve, nextIndex , nextIndex + 1, intakeCommand, shootCommand).asProxy();
    }
    SmartDashboard.putBoolean("isFinished", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
