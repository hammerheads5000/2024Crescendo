// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LightConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.Swerve;

public class AlignToNoteCommand extends Command {
  Swerve swerve;
  DoubleSubscriber angleSubscriber;
  BooleanSubscriber hasTargetSubscriber;
  Rotation2d desiredRotation;
  Timer timer;
  boolean timingAlignment = false;
  LightsSubsystem lightsSubsystem;
  /** Creates a new AlignToNoteCommand. */
  public AlignToNoteCommand(Swerve swerve, LightsSubsystem lightsSubsystem) {
    this.swerve = swerve;
    this.lightsSubsystem = lightsSubsystem;
    angleSubscriber = VisionConstants.noteYawTopic.subscribe(0.0);
    hasTargetSubscriber = VisionConstants.colorHasTargetsTopic.subscribe(false);
    desiredRotation = swerve.getPose().getRotation();
    timer = new Timer();

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakeConstants.noteAlignmentPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!hasTargetSubscriber.get()) 
    {
      lightsSubsystem.setSolidColor(LightConstants.RED);
      return;
    }
    lightsSubsystem.setSolidColor(LightConstants.BLUE);
    Rotation2d robotAngle = swerve.getPose().getRotation();
    Rotation2d robotToNoteRotation = Rotation2d.fromDegrees(-angleSubscriber.get());
    desiredRotation = robotAngle.rotateBy(robotToNoteRotation);
    // System.out.print(robotAngle.getDegrees());
    // System.out.print(robotToNoteRotation.getDegrees());
    // System.out.println(desiredRotation.getDegrees());
    swerve.driveFacingNote(
        MetersPerSecond.zero(),
        MetersPerSecond.zero(),
        desiredRotation); // turn to face note
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lightsSubsystem.setSolidColor(LightConstants.BLANK);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!hasTargetSubscriber.get()) return false;

    boolean aligned = Math.abs(angleSubscriber.get()) <= IntakeConstants.noteAlignTolerance.in(Degrees);
    if (aligned && !timingAlignment) {
      timer.restart();
      timingAlignment = true;
    }
    else if (!aligned && timingAlignment) {
      timingAlignment = false;
    }
    return hasTargetSubscriber.get() && aligned && timer.hasElapsed(IntakeConstants.alignedDelay.in(Seconds));
  }

  public Measure<Distance> distanceFromPitch(double pitch) {
    double cameraHeight = VisionConstants.robotToNoteDetectionCam.getZ(); // meters
    double cameraPitch = -VisionConstants.robotToNoteDetectionCam.getRotation().getY(); // radians

    double distance = cameraHeight * Math.tan(Math.PI/2 - cameraPitch - pitch);
    return Meters.of(distance);
  }
}
