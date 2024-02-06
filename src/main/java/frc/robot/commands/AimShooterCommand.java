// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;

public class AimShooterCommand extends Command {
  Swerve swerve;
  CommandXboxController controller;
  Translation3d speakerPos;
  ShooterSubsystem shooterSubsystem;
  Measure<Distance> distanceToSpeaker;

  /** Creates a new TeleopSwerve. */
  public AimShooterCommand(Swerve swerve, CommandXboxController controller, ShooterSubsystem shooterSubsystem, Measure<Angle> angle) {
    this.swerve = swerve;
    this.controller = controller;
    this.shooterSubsystem = shooterSubsystem;

    // set speaker position
    Optional<Alliance> team = DriverStation.getAlliance();
    if (team.isPresent() && team.get() == Alliance.Red){
      speakerPos = FieldConstants.redSpeakerPos;
    }else{ //auto blue if there is no team because why not
      speakerPos = FieldConstants.blueSpeakerPos;
    }

    this.distanceToSpeaker = distanceFromAngle(angle);

    addRequirements(swerve, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d speakerToRobot = swerve.getPose().getTranslation().minus(speakerPos.toTranslation2d());
    Translation2d velocityVec = speakerToRobot.rotateBy(new Rotation2d(Degrees.of(90))); // use tangent as velocity
    velocityVec.div(velocityVec.getNorm()); // normalize

    velocityVec.times(SwerveConstants.maxDriveSpeed.times(
        Math.abs(controller.getLeftX()) >= SwerveConstants.controllerDeadband
            ? -controller.getLeftX() : 0).in(MetersPerSecond)); // scale to speed

    Rotation2d angleToFace = speakerToRobot.getAngle().unaryMinus();

    swerve.driveFacingAngle(
        MetersPerSecond.of(velocityVec.getX()), 
        MetersPerSecond.of(velocityVec.getY()),
        angleToFace);
  }

  private Measure<Distance> distanceFromAngle(Measure<Angle> angle) {
    double v = ShooterConstants.exitVelocity.in(MetersPerSecond);
    double g = ShooterConstants.gravity.in(MetersPerSecondPerSecond);
    double theta = angle.in(Radians);
    double h = speakerPos.getZ();

    return Meters.of(v*v/g * Math.sin(theta)*Math.cos(theta)
        - v/g * Math.cos(theta) * Math.sqrt(v*v*Math.sin(theta)-2*g*h)); // formula!
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
