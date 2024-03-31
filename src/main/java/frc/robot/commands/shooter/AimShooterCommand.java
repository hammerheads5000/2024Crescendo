// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LightConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.ShooterHeightPIDSubsystem;

public class AimShooterCommand extends Command {
  Swerve swerve;
  CommandXboxController controller;
  Translation3d speakerPos;
  ShooterHeightPIDSubsystem shooterHeightPIDSubsystem;
  LightsSubsystem lightsSubsystem;
  boolean aligned = false;

  BooleanPublisher alignedToSpeakerPublisher;

  /**
   * Construct AimShooterCommand with controller control
   * @param swerve
   * @param controller
   * @param shooterHeightPIDSubsystem
   * @param lightsSubsystem
   */
  public AimShooterCommand(Swerve swerve, CommandXboxController controller, ShooterHeightPIDSubsystem shooterHeightPIDSubsystem, LightsSubsystem lightsSubsystem) {
    this(swerve, shooterHeightPIDSubsystem, lightsSubsystem);
    this.controller = controller;
  }

  /**
   * Construct AimShooterCommand without controller control
   * @param swerve
   * @param shooterHeightPIDSubsystem
   */
  public AimShooterCommand(Swerve swerve, ShooterHeightPIDSubsystem shooterHeightPIDSubsystem, LightsSubsystem lightsSubsystem) {
    this.swerve = swerve;
    this.shooterHeightPIDSubsystem = shooterHeightPIDSubsystem;
    this.lightsSubsystem = lightsSubsystem;
    
    // set speaker position
    Optional<Alliance> team = DriverStation.getAlliance();
    if (team.isPresent() && team.get() == Alliance.Red){
      speakerPos = FieldConstants.redSpeakerPos;
    }else{ //auto blue if there is no team because why not
      speakerPos = FieldConstants.blueSpeakerPos;
    }
    
    alignedToSpeakerPublisher = LoggingConstants.alignedToSpeakerPublisher;

    addRequirements(swerve, shooterHeightPIDSubsystem);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterHeightPIDSubsystem.enable();
    SwerveConstants.headingPID.reset();
    
    // set speaker position
    Optional<Alliance> team = DriverStation.getAlliance();
    if (team.isPresent() && team.get() == Alliance.Red){
      speakerPos = FieldConstants.redSpeakerPos;
    }else{ //auto blue if there is no team because why not
      speakerPos = FieldConstants.blueSpeakerPos;
    }

    lightsSubsystem.setSolidColor(LightConstants.ORANGE);
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

    Measure<Distance> distance = getDistanceToSpeaker();
    Measure<Velocity<Distance>> velocity = velocityToShoot(distance);
    Measure<Angle> shooterAngle = angleFromDistanceAndVelocity(distance, velocity);
    shooterHeightPIDSubsystem.setTargetAngle(shooterAngle);

    LoggingConstants.shooterSpeedRequestPublisher.set(linearToAngularVelocity(velocity).in(RPM));

    // alignment
    Translation2d robotHeadingVec = new Translation2d(Meters.of(-1), Meters.zero()).rotateBy(swerve.getPose().getRotation()); // points in direction of shot
    Translation2d robotToDestination = robotHeadingVec.times(speakerToRobot.getNorm());
    Translation2d destinationPos = swerve.getPose().getTranslation().plus(robotToDestination); // where the note would reach distance of speaker
    aligned = Meters.of(speakerPos.toTranslation2d().getDistance(destinationPos)).lte(ShooterConstants.readyAlignTolerance);
    alignedToSpeakerPublisher.set(aligned);

    if (aligned && LoggingConstants.shooterAimedSubscriber.get() && LoggingConstants.shooterAtSpeedSubscriber.get()) {
      lightsSubsystem.setSolidColor(LightConstants.GREEN);
    }
  }

  private Translation2d getApproachVelocityVec(Translation2d speakerToRobot) {
    Translation2d approachVelocityVec = new Translation2d().plus(speakerToRobot).div(speakerToRobot.getNorm()); // unit vector away from speaker
  
    approachVelocityVec = approachVelocityVec.times(SwerveConstants.defaultDriveSpeed.times(
      Math.abs(controller.getLeftY()) >= Constants.controllerDeadband
            ? controller.getLeftY() : 0).in(MetersPerSecond)); // scale to speed
    return approachVelocityVec;
  }

  private Translation2d getStrafeVelocityVec(Translation2d speakerToRobot) {
    Translation2d strafeVelocityVec = new Translation2d().plus(speakerToRobot).rotateBy(new Rotation2d(Degrees.of(90))); // use tangent as velocity
    strafeVelocityVec = strafeVelocityVec.div(strafeVelocityVec.getNorm()); // normalize
    strafeVelocityVec = strafeVelocityVec.times(SwerveConstants.defaultDriveSpeed.times(
        Math.abs(controller.getLeftX()) >= Constants.controllerDeadband
            ? controller.getLeftX() : 0).in(MetersPerSecond)); // scale to speed
    return strafeVelocityVec;
  }

  public boolean isAligned() {
    return aligned;
  }

  private Measure<Angle> angleFromDistance(Measure<Distance> distance) {
    return angleFromDistanceAndVelocity(distance, ShooterConstants.exitVelocity);
  }

  private Measure<Angle> angleFromDistanceAndVelocity(Measure<Distance> distance, Measure<Velocity<Distance>> velocity) {
    double v = velocity.in(MetersPerSecond);
    double g = ShooterConstants.gravity.in(MetersPerSecondPerSecond);
    double d = distance.in(Meters);
    double h = speakerPos.getZ();
    
    double r = Math.sqrt(d*d + h*h); // linear distance to opening
    double a = Math.atan2(h, d);
    // got formula from https://iitutor.com/the-trajectory-of-projectile-motion-on-an-inclined-plane-with-maximum-range-explained/
    // from part 4, solving for theta (with wolfram alpha)
    return Radians.of(0.5 * Math.asin( (r*g*Math.pow(Math.cos(a),2)) / (v*v) + Math.sin(a)) + a/2); // formula!
  }

  private Measure<Velocity<Angle>> linearToAngularVelocity(Measure<Velocity<Distance>> velocity) {
    double proportion = velocity.in(MetersPerSecond) / ShooterConstants.exitVelocity.in(MetersPerSecond);
    return ShooterConstants.topSpeed.times(proportion);
  }

  private Measure<Velocity<Distance>> minimumVelocity(Measure<Distance> distance) {
    double g = ShooterConstants.gravity.in(MetersPerSecondPerSecond);
    double d = distance.in(Meters);
    double h = speakerPos.getZ();

    return MetersPerSecond.of(Math.sqrt(g * (d*d + 4*h*h) / (2*h))); // formula!
  }

  private Measure<Velocity<Distance>> velocityToShoot(Measure<Distance> distance) {
    double minVel = minimumVelocity(distance).in(MetersPerSecond);
    double maxVel = ShooterConstants.exitVelocity.in(MetersPerSecond);
    double gain = ShooterConstants.variableVelocityGain.in(MetersPerSecond);

    return MetersPerSecond.of(Math.min(minVel+gain, maxVel));
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
    aligned = false;
    alignedToSpeakerPublisher.set(false);
    
    if (controller != null)
      LoggingConstants.shooterSpeedRequestPublisher.set(ShooterConstants.manualShooterSpeed.in(RPM));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}