// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class ColoredTargetAutomatedSwerve extends Command {
  /** Creates a new ColoredTargetAutomatedSwerve. */
  DoubleArraySubscriber TargetPoseSub;
  double[] blank = new double[] {0,0,0,0,0,0,0};
  Swerve swerve;
  CommandXboxController controller;
  DoubleArrayTopic TargetPoseTopic;
  BooleanTopic HasTargetsTopic;
  DoubleArraySubscriber TargetPoseSubScriber;
  BooleanSubscriber HasTargetsSubscriber;
  public ColoredTargetAutomatedSwerve(DoubleArrayTopic TargetPoseTopic, BooleanTopic HasTargetsTopic, Swerve swerve, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.TargetPoseTopic = TargetPoseTopic;
    this.HasTargetsTopic = HasTargetsTopic;
    this.swerve = swerve;
    this.controller = controller;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    TargetPoseSubScriber = TargetPoseTopic.subscribe(blank);
    HasTargetsSubscriber = HasTargetsTopic.subscribe(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(HasTargetsSubscriber.get())
    {
      double[] TargetPose = TargetPoseSubScriber.get();
      double YAngle = TargetPose[5];
      double AngleRequest = YAngle >= Constants.SwerveConstants.ColoredTargetAngleDeadband ? YAngle : 0 ;
      double ControllerRequest = (Math.abs(controller.getLeftY()) >= Constants.SwerveConstants.controllerDeadband ? controller.getLeftY() : 0);
      double CurrentAngle = Constants.SwerveConstants.drivetrain.getState().Pose.getRotation().getDegrees();
      swerve.drive(
      SwerveConstants.maxDriveSpeed.times(/*ControllerRequest * Math.cos(CurrentAngle)*/ 0),
      SwerveConstants.maxDriveSpeed.times( /*ControllerRequest * Math.sin(CurrentAngle) */ 0), 
      SwerveConstants.maxRotSpeed.times(AngleRequest));
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
