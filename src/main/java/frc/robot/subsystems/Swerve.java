// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.EnumSet;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.UnitConstants;
import frc.robot.Constants.VisionConstants;

public class Swerve extends SubsystemBase {
  private SwerveDrivetrain drivetrain;
  private SwerveRequest.FieldCentric fieldCentricRequest;
  private SwerveRequest.RobotCentric robotCentricRequest;
  private SwerveRequest.ApplyChassisSpeeds chassisSpeedsRequest;

  private DoubleArraySubscriber aprilTagSubscriber;
  StructArrayPublisher<SwerveModuleState> statesPublisher = LoggingConstants.moduleStatesPublisher;
  StructArrayPublisher<SwerveModuleState> desiredStatesPublisher = LoggingConstants.desiredModuleStatesPublisher;
  DoublePublisher rotationPublisher = LoggingConstants.rotationPublisher;
  DoubleArrayPublisher chassisSpeedsPublisher = LoggingConstants.chassisSpeedsPublisher;

  private Field2d field = new Field2d();

  /** Creates a new Swerve. */
  public Swerve() {
    drivetrain = SwerveConstants.drivetrain;
    drivetrain.configNeutralMode(SwerveConstants.driveNeutralMode);

    drivetrain.getPigeon2().getConfigurator().apply(Constants.pigeonMountConfigs);

    // apply current limits
    for (int i = 0; i < 4; i++) {
      SwerveModule module = drivetrain.getModule(i);
      module.getDriveMotor().getConfigurator().apply(SwerveConstants.driveCurrentLimits);
      module.getSteerMotor().getConfigurator().apply(SwerveConstants.angleCurrentLimits);
    }

    // setup drive requests
    fieldCentricRequest = new SwerveRequest.FieldCentric()
        .withDeadband(SwerveConstants.velocityDeadband.in(MetersPerSecond))
        .withRotationalDeadband(SwerveConstants.rotationDeadband.in(RadiansPerSecond))
        .withDriveRequestType(SwerveConstants.driveRequestType)
        .withSteerRequestType(SwerveConstants.steerRequestType);

    robotCentricRequest = new SwerveRequest.RobotCentric()
        .withDeadband(SwerveConstants.velocityDeadband.in(MetersPerSecond))
        .withRotationalDeadband(SwerveConstants.rotationDeadband.in(RadiansPerSecond))
        .withDriveRequestType(SwerveConstants.driveRequestType)
        .withSteerRequestType(SwerveConstants.steerRequestType);
        
    chassisSpeedsRequest = new SwerveRequest.ApplyChassisSpeeds()
        .withDriveRequestType(SwerveConstants.driveRequestType)
        .withSteerRequestType(SwerveConstants.steerRequestType);
        
    // setup heading pid
    SwerveConstants.headingPID.enableContinuousInput(-Math.PI, Math.PI);
    SwerveConstants.headingPID.setTolerance(SwerveConstants.rotationalPIDTolerance.in(Radians));
        
    aprilTagSubscriber = VisionConstants.poseTopic.subscribe(new double[3]);

    // creates listener such that when the pose estimate NetworkTables topic
    //  is updated, it calls applyVisionMeasurement to update pose
    NetworkTableListener.createListener(
        aprilTagSubscriber,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll), // listens for any value change
        event -> {
          applyVisionMeasurement(event.valueData.value.getDoubleArray(), event.valueData.value.getTime());
        });

    SmartDashboard.putData("Field", field);
    drivetrain.setVisionMeasurementStdDevs(VisionConstants.stdDvsMatrix);
  }
    
  /**
   * Drive robot with respect to field
   * 
   * @param xVel field relative forward velocity
   * @param yVel field relative left velocity
   * @param rot  angular velocity counterclockwise
   */
  public void driveFieldCentric(Measure<Velocity<Distance>> xVel, Measure<Velocity<Distance>> yVel,
      Measure<Velocity<Angle>> rot) {
    drivetrain.setControl(
        fieldCentricRequest.withVelocityX(xVel.in(MetersPerSecond))
            .withVelocityY(yVel.in(MetersPerSecond))
            .withRotationalRate(rot.in(RadiansPerSecond)));
  }

  /**
   * Drive robot while facing angle
   * 
   * @param xVel field centric forward velocity
   * @param yVel field centric left velocity
   * @param angle angle to face (field centric)
   */
  public void driveFacingAngle(Measure<Velocity<Distance>> xVel, Measure<Velocity<Distance>> yVel, Rotation2d angle) {
    // calculate rotational velocity with pid (radians per second)
    double omega = SwerveConstants.headingPID.calculate(
        getPose().getRotation().getRadians(),
        angle.getRadians());

    drivetrain.setControl(fieldCentricRequest
        .withVelocityX(xVel.in(MetersPerSecond))
        .withVelocityY(yVel.in(MetersPerSecond))
        .withRotationalRate(omega));
  }

  /**
   * Drive robot with respect to robot
   * 
   * @param xVel robot forward velocity
   * @param yVel robot left velocity
   * @param rot  angular velocity counterclockwise
   */
  public void driveRobotCentric(Measure<Velocity<Distance>> xVel, Measure<Velocity<Distance>> yVel,
      Measure<Velocity<Angle>> rot) {
    drivetrain.setControl(
        robotCentricRequest.withVelocityX(xVel.in(MetersPerSecond))
            .withVelocityY(yVel.in(MetersPerSecond))
            .withRotationalRate(rot.in(RadiansPerSecond)));
  }

  /**
   * Drive robot with respect to robot
   * 
   * @param chassisSpeeds chassis speeds to set
   */
  public void driveRobotCentric(ChassisSpeeds chassisSpeeds) {
    drivetrain.setControl(chassisSpeedsRequest.withSpeeds(chassisSpeeds));
  }

  /**
   * Get robot centric speeds
   * @return robot centric chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return SwerveConstants.kinematics.toChassisSpeeds(drivetrain.getState().ModuleTargets);
  }

  /**
   * Reset swerve odometry/pose tracking to 0,0
   */
  public void resetPose() {
    drivetrain.seedFieldRelative();
  }

  public void resetPose(Pose2d pose) {
    drivetrain.seedFieldRelative(pose);
  }

  /**
   * Gets calculated pose (from odometry with any added vision measurements)
   * 
   * @return pose
   */
  public Pose2d getPose() {
    return drivetrain.getState().Pose;
  }

  /**
   * Applies estimated pose from AprilTags to pose estimation
   * 
   * @param estimatedRobotPose Returned from {@link org.photonvision.PhotonPoseEstimator#update() photonPoseEstimator.update()}
   * Contains a Pose3d and a timestamp
   */
  public void applyVisionMeasurement(EstimatedRobotPose estimatedRobotPose) {
    drivetrain.seedFieldRelative(getPose());
    drivetrain.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
  }

  /**
   * Applies vision measurement from NetworkTables data
   * 
   * @param poseArray double array storing the data of a Pose2d of format {x (meters), y (meters), rotation (radians)}
   * @param timestamp timestamp in microseconds
   */
  public void applyVisionMeasurement(double[] poseArray, long timestamp) {
    // converts array of format {x (m), y (m), rotation (rad)} to Pose2d
    Pose2d pose = new Pose2d(poseArray[0], poseArray[1], new Rotation2d(poseArray[2]));
    double timestampSeconds = timestamp * UnitConstants.microsecondsToSeconds; // convert microseconds timestamp to seconds
    drivetrain.addVisionMeasurement(pose, timestampSeconds);
  }

  @Override
  public void periodic() {
    field.setRobotPose(getPose());
    statesPublisher.set(drivetrain.getState().ModuleStates);
    desiredStatesPublisher.set(drivetrain.getState().ModuleTargets);
    rotationPublisher.set(Degrees.of(drivetrain.getPigeon2().getAngle()).in(Radians));
  }
}
