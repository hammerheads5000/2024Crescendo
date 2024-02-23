// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import javax.xml.validation.SchemaFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;

public class AlignToNoteCommand extends Command {
  Swerve swerve;
  DoubleSubscriber angleSubscriber;
  BooleanSubscriber targetSubscriber;
  /** Creates a new AlignToNoteCommand. */
  public AlignToNoteCommand(Swerve swerve) {
    this.swerve = swerve;

    angleSubscriber = VisionConstants.noteYawTopic.subscribe(0.0);
    targetSubscriber = VisionConstants.colorHasTargetsTopic.subscribe(false);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(targetSubscriber.get()){
    Rotation2d robotAngle = swerve.getPose().getRotation();
    Rotation2d robotToNoteRotation = Rotation2d.fromDegrees(-angleSubscriber.get());
    swerve.driveRobotCentric(
        MetersPerSecond.zero(),
        MetersPerSecond.zero(),
        DegreesPerSecond.of(-angleSubscriber.get() * 2)); // turn to face note
        SmartDashboard.putNumber("-angleSubscriber", -angleSubscriber.get());
    } 
      SmartDashboard.putBoolean("HasTarget", targetSubscriber.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(targetSubscriber.get())
    {
    return Math.abs(angleSubscriber.get()) <= SwerveConstants.noteRotationalTolerance.in(Degrees);
    }
    else
    {
      return false;
    }
  }
}
