// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;

public class AlignToNoteCommand extends Command {
  Swerve swerve;
  DoubleSubscriber angleSubscriber;
  BooleanSubscriber hasTargetSubscriber;
  Rotation2d desiredRotation;

  /** Creates a new AlignToNoteCommand. */
  public AlignToNoteCommand(Swerve swerve) {
    this.swerve = swerve;

    angleSubscriber = VisionConstants.noteYawTopic.subscribe(0.0);
    hasTargetSubscriber = VisionConstants.colorHasTargetsTopic.subscribe(false);
    desiredRotation = swerve.getPose().getRotation();

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!hasTargetSubscriber.get()) return;
    
    Rotation2d robotAngle = swerve.getPose().getRotation();
    Rotation2d robotToNoteRotation = Rotation2d.fromDegrees(-angleSubscriber.get());
    desiredRotation = robotAngle.rotateBy(robotToNoteRotation);

    swerve.driveFacingAngle(
        MetersPerSecond.zero(),
        MetersPerSecond.zero(),
        desiredRotation); // turn to face note
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasTargetSubscriber.get() && Math.abs(angleSubscriber.getAsDouble()) <= IntakeConstants.noteRotationalTolerance.in(Degrees);
  }
}
