// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;

public class AlignToNoteCommand extends Command {
  Swerve swerve;
  DoubleSubscriber angleSubscriber;
  BooleanSubscriber hasTargetSubscriber;
  DoubleSubscriber pitchSubscriber;
  Rotation2d desiredRotation;

  /** Creates a new AlignToNoteCommand. */
  public AlignToNoteCommand(Swerve swerve) {
    this.swerve = swerve;

    angleSubscriber = VisionConstants.noteYawTopic.subscribe(0.0);
    hasTargetSubscriber = VisionConstants.colorHasTargetsTopic.subscribe(false);
    pitchSubscriber = VisionConstants.notePitchTopic.subscribe(0.0);
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
    double distanceToNote = distanceFromPitch().in(Meters);
    Translation2d projectedNote = new Translation2d(distanceToNote, 0).rotateBy(swerve.getPose().getRotation());
    Translation2d actualNote = new Translation2d(distanceToNote, 0).rotateBy(desiredRotation);
    return hasTargetSubscriber.get() && projectedNote.getDistance(actualNote) <= IntakeConstants.noteAlignTolerance.in(Meters);
  }

  private Measure<Distance> distanceFromPitch() {
    double pitch = Degrees.of(pitchSubscriber.get()).in(Radians);
    double cameraHeight = VisionConstants.robotToNoteDetectionCam.getZ(); // meters
    double cameraPitch = VisionConstants.robotToNoteDetectionCam.getRotation().getY(); // radians

    double distance = cameraHeight * Math.tan(Math.PI/2 - cameraPitch - pitch);
    return Meters.of(distance);
  }
}
