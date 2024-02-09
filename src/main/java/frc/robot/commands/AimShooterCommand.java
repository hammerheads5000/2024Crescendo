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

import edu.wpi.first.math.controller.PIDController;
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
  Measure<Distance> targetDistance;
  PIDController distancePID;
  boolean isTargetDistanceFinal;

  /**
   * Constructor with automatic selection of distance
   * @param swerve
   * @param controller
   * @param shooterSubsystem
   */
  public AimShooterCommand(Swerve swerve, CommandXboxController controller, ShooterSubsystem shooterSubsystem) {
    this.swerve = swerve;
    this.controller = controller;
    this.shooterSubsystem = shooterSubsystem;
    this.distancePID = ShooterConstants.moveToDistancePID;
    this.isTargetDistanceFinal = false;

    // set speaker position
    Optional<Alliance> team = DriverStation.getAlliance();
    if (team.isPresent() && team.get() == Alliance.Red){
      speakerPos = FieldConstants.redSpeakerPos;
    }else{ //auto blue if there is no team because why not
      speakerPos = FieldConstants.blueSpeakerPos;
    }

    addRequirements(swerve, shooterSubsystem);
  }

  /**
   * Constructor for fixed distance
   * @param swerve
   * @param controller
   * @param shooterSubsystem
   * @param isClose set to true for close shooting, false for far
   */
  public AimShooterCommand(Swerve swerve, CommandXboxController controller, ShooterSubsystem shooterSubsystem, boolean isClose) {
    this(swerve, controller, shooterSubsystem);

    this.targetDistance = isClose ? distanceFromAngle(ShooterConstants.closeAngle) : distanceFromAngle(ShooterConstants.farAngle);
    this.isTargetDistanceFinal = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.start();
    if (!isTargetDistanceFinal) { targetDistance = getClosestDistance(); };
    distancePID.setSetpoint(targetDistance.in(Meters));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d speakerToRobot = swerve.getPose().getTranslation().minus(speakerPos.toTranslation2d());

    // strafing
    Translation2d strafeVelocityVec = new Translation2d().plus(speakerToRobot).rotateBy(new Rotation2d(Degrees.of(90))); // use tangent as velocity
    strafeVelocityVec = strafeVelocityVec.div(strafeVelocityVec.getNorm()); // normalize
    strafeVelocityVec = strafeVelocityVec.times(SwerveConstants.maxDriveSpeed.times(
        Math.abs(controller.getLeftX()) >= SwerveConstants.controllerDeadband
            ? -controller.getLeftX() : 0).in(MetersPerSecond)); // scale to speed
    
    // approaching to correct distance
    Translation2d approachVelocityVec = new Translation2d().plus(speakerToRobot).div(speakerToRobot.getNorm()); // unit vector away from speaker
    approachVelocityVec.rotateBy(new Rotation2d(Degrees.of(180))); // unit vector towards from speaker
    approachVelocityVec = approachVelocityVec.times(distancePID.calculate(getDistanceToSpeaker().in(Meters))); // scale with PID

    // driving
    Translation2d totalVelocityVec = strafeVelocityVec.plus(approachVelocityVec);
    Rotation2d angleToFace = speakerToRobot.getAngle();

    swerve.driveFacingAngle(
        MetersPerSecond.of(totalVelocityVec.getX()), 
        MetersPerSecond.of(totalVelocityVec.getY()),
        angleToFace);

    SmartDashboard.putNumber("Distance to Speaker", getDistanceToSpeaker().in(Meters));
    SmartDashboard.putNumber("Target distance", targetDistance.in(Meters));
  }

  private Measure<Distance> distanceFromAngle(Measure<Angle> angle) {
    double v = ShooterConstants.exitVelocity.in(MetersPerSecond);
    double g = ShooterConstants.gravity.in(MetersPerSecondPerSecond);
    double theta = angle.in(Radians);
    double h = speakerPos.getZ();

    return Meters.of(v*v/g * Math.sin(theta)*Math.cos(theta)
        - v/g * Math.cos(theta) * Math.sqrt(v*v*Math.sin(theta)-2*g*h)); // formula!
  }

  /**
   * Determines which angle/distance to shoot at
   * @return Distance to speaker closest to current position
   */
  private Measure<Distance> getDistanceToSpeaker() {
    return Meters.of(swerve.getPose().getTranslation().getDistance(speakerPos.toTranslation2d()));
  }

  private Measure<Distance> getClosestDistance() {
    Measure<Distance> closeDistance = distanceFromAngle(ShooterConstants.closeAngle);
    Measure<Distance> farDistance = distanceFromAngle(ShooterConstants.farAngle);
    Measure<Distance> currentDistance = getDistanceToSpeaker();

    // check first if closer than close or farther than far
    if (currentDistance.lte(closeDistance)) {
      return closeDistance;
    }
    if (currentDistance.gte(farDistance)) {
      return farDistance;
    }

    Measure<Distance> distToClose = currentDistance.minus(closeDistance);
    Measure<Distance> distToFar = farDistance.minus(currentDistance);

    return distToClose.lte(distToFar) ? closeDistance : farDistance; // return closer distance
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
