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
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class ColoredTargetAutomatedSwerve extends Command {
  /** Creates a new ColoredTargetAutomatedSwerve. */
  DoubleArraySubscriber TargetPoseSub;
  double[] blank = new double[] {0,0,0,0,0,0,0};
  Swerve swerve;
  DoubleArrayTopic TargetPoseTopic;
  BooleanTopic HasTargetsTopic;
  DoubleArraySubscriber TargetPoseSubScriber;
  BooleanSubscriber HasTargetsSubscriber;
  public ColoredTargetAutomatedSwerve(DoubleArrayTopic TargetPoseTopic, BooleanTopic HasTargetsTopic, Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.TargetPoseTopic = TargetPoseTopic;
    this.HasTargetsTopic = HasTargetsTopic;
    this.swerve = swerve;
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
      double Yangle = TargetPose[5];
      swerve.drive(SwerveConstants.maxDriveSpeed.times(0),SwerveConstants.maxDriveSpeed.times(0),  SwerveConstants.maxRotSpeed.times(Yangle));
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
