// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;

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
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.ShooterHeightPIDSubsystem;

public class AimShooterCommand extends Command {
  Swerve swerve;
  CommandXboxController controller;
  Translation3d speakerPos;
  ShooterHeightPIDSubsystem shooterHeightPIDSubsystem;
  boolean aligned = false;

  /**
   * Construct AimShooterCommand with controller control
   * @param swerve
   * @param controller
   * @param shooterHeightPIDSubsystem
   */
  public AimShooterCommand(Swerve swerve, CommandXboxController controller, ShooterHeightPIDSubsystem shooterHeightPIDSubsystem) {
    this.swerve = swerve;
    this.controller = controller;
    this.shooterHeightPIDSubsystem = shooterHeightPIDSubsystem;
    shooterHeightPIDSubsystem.enable();

    // set speaker position
    Optional<Alliance> team = DriverStation.getAlliance();
    if (team.isPresent() && team.get() == Alliance.Red){
      speakerPos = FieldConstants.redSpeakerPos;
    }else{ //auto blue if there is no team because why not
      speakerPos = FieldConstants.blueSpeakerPos;
    }

    addRequirements(swerve, shooterHeightPIDSubsystem);
  }

  /**
   * Construct AimShooterCommand without controller control
   * @param swerve
   * @param shooterHeightPIDSubsystem
   */
  public AimShooterCommand(Swerve swerve, ShooterHeightPIDSubsystem shooterHeightPIDSubsystem) {
    this.swerve = swerve;
    this.shooterHeightPIDSubsystem = shooterHeightPIDSubsystem;
    shooterHeightPIDSubsystem.enable();

    // set speaker position
    Optional<Alliance> team = DriverStation.getAlliance();
    if (team.isPresent() && team.get() == Alliance.Red){
      speakerPos = FieldConstants.redSpeakerPos;
    }else{ //auto blue if there is no team because why not
      speakerPos = FieldConstants.blueSpeakerPos;
    }

    addRequirements(swerve, shooterHeightPIDSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterHeightPIDSubsystem.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d speakerToRobot = swerve.getPose().getTranslation().minus(speakerPos.toTranslation2d());
    Rotation2d angleToFace = speakerToRobot.getAngle();
    
    // strafing
    Translation2d strafeVelocityVec = controller != null ? getStrafeVelocityVec(speakerToRobot) : new Translation2d();
    
    // approaching to correct distance
    Translation2d approachVelocityVec = controller != null ? getApproachVelocityVec(speakerToRobot) : new Translation2d();

    // driving
    Translation2d totalVelocityVec = strafeVelocityVec.plus(approachVelocityVec);

    swerve.driveFacingAngle(
        MetersPerSecond.of(totalVelocityVec.getX()), 
        MetersPerSecond.of(totalVelocityVec.getY()),
        angleToFace);

    Measure<Angle> shooterAngle = angleFromDistance(getDistanceToSpeaker());
    shooterHeightPIDSubsystem.setTargetAngle(shooterAngle);

    // alignment
    Translation2d robotHeadingVec = new Translation2d(Meters.of(-1), Meters.zero()).rotateBy(swerve.getPose().getRotation()); // points in direction of shot
    Translation2d robotToDestination = robotHeadingVec.times(speakerToRobot.getNorm());
    Translation2d destinationPos = swerve.getPose().getTranslation().plus(robotToDestination); // where the note would reach distance of speaker
    aligned = Meters.of(speakerPos.toTranslation2d().getDistance(destinationPos)).lte(ShooterConstants.readyAlignTolerance);
    SmartDashboard.putBoolean("Aligned To Speaker", aligned);
  }

  private Translation2d getApproachVelocityVec(Translation2d speakerToRobot) {
    Translation2d approachVelocityVec = new Translation2d().plus(speakerToRobot).div(speakerToRobot.getNorm()); // unit vector away from speaker
  
    approachVelocityVec = approachVelocityVec.times(SwerveConstants.maxDriveSpeed.times(
      Math.abs(controller.getLeftY()) >= Constants.controllerDeadband
            ? -controller.getLeftY() : 0).in(MetersPerSecond)); // scale to speed
    return approachVelocityVec;
  }

  private Translation2d getStrafeVelocityVec(Translation2d speakerToRobot) {
    Translation2d strafeVelocityVec = new Translation2d().plus(speakerToRobot).rotateBy(new Rotation2d(Degrees.of(90))); // use tangent as velocity
    strafeVelocityVec = strafeVelocityVec.div(strafeVelocityVec.getNorm()); // normalize
    strafeVelocityVec = strafeVelocityVec.times(SwerveConstants.maxDriveSpeed.times(
        Math.abs(controller.getLeftX()) >= Constants.controllerDeadband
            ? -controller.getLeftX() : 0).in(MetersPerSecond)); // scale to speed
    return strafeVelocityVec;
  }

  public boolean isAligned() {
    return aligned;
  }

  private Measure<Angle> angleFromDistance(Measure<Distance> distance) {
    double v = ShooterConstants.exitVelocity.in(MetersPerSecond);
    double g = ShooterConstants.gravity.in(MetersPerSecondPerSecond);
    double d = distance.in(Meters);
    double h = speakerPos.getZ();

    double a = Math.atan2(h, d);
    
    return Radians.of(0.5 * Math.asin( (d*g*Math.pow(Math.cos(a),2)) / (v*v) + Math.sin(a)) + a/2); // formula!
  }

  /**
   * Determines which angle/distance to shoot at
   * @return Distance to speaker closest to current position
   */
  private Measure<Distance> getDistanceToSpeaker() {
    return Meters.of(swerve.getPose().getTranslation().getDistance(speakerPos.toTranslation2d()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //shooterHeightPIDSubsystem.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
