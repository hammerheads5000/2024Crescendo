// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.EnumSet;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.UnitConstants;
import frc.robot.Constants.VisionConstants;

public class Swerve extends SubsystemBase {
  private SwerveDrivetrain drivetrain;
  private SwerveRequest.FieldCentric fieldCentricRequest;
  private SwerveRequest.RobotCentric robotCentricRequest;
  private SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest;

  private DoubleArraySubscriber aprilTagSubscriber;
  private Pose3d speakerPose;

  /** Creates a new Swerve. */
  public Swerve() {
    drivetrain = SwerveConstants.drivetrain;
    drivetrain.configNeutralMode(SwerveConstants.driveNeutralMode);

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

    fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(SwerveConstants.velocityDeadband.in(MetersPerSecond))
        .withRotationalDeadband(SwerveConstants.rotationDeadband.in(RadiansPerSecond))
        .withDriveRequestType(SwerveConstants.driveRequestType)
        .withSteerRequestType(SwerveConstants.steerRequestType);

    fieldCentricFacingAngleRequest.HeadingController = 

    aprilTagSubscriber = VisionConstants.poseTopic.subscribe(new double[3]);

    Optional<Alliance> team = DriverStation.getAlliance();
    if (team.isPresent() && team.get() == Alliance.Red){
      speakerPose = SwerveConstants.redSpeakerPose;
    }else{ //auto blue if there is no team because why not
      speakerPose = SwerveConstants.blueSpeakerPose;
    }
    
    // creates listener such that when the pose estimate NetworkTables topic
    //  is updated, it calls applyVisionMeasurement to update pose
    NetworkTableListener.createListener(
        aprilTagSubscriber,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll), // listens for any value change
        event -> {
          applyVisionMeasurement(event.valueData.value.getDoubleArray(), event.valueData.value.getTime());
        });
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
    // apply request with params
    drivetrain.setControl(
        fieldCentricRequest.withVelocityX(xVel.in(MetersPerSecond))
            .withVelocityY(yVel.in(MetersPerSecond))
            .withRotationalRate(rot.in(RadiansPerSecond)));
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
    // apply request with params
    drivetrain.setControl(
        robotCentricRequest.withVelocityX(xVel.in(MetersPerSecond))
            .withVelocityY(yVel.in(MetersPerSecond))
            .withRotationalRate(rot.in(RadiansPerSecond)));
  }

  /**
   * Drive robot with respect to the field but facing the speakers
   * 
   * @param xVel robot forward velocity
   * @param yVel robot left velocity
   */
  public void driveFieldCentricFacingAngle(Measure<Velocity<Distance>> xVel, Measure<Velocity<Distance>> yVel) {
    // apply request with params
    Rotation2d targetAngle = new Rotation2d(speakerPose.getX() - getPose().getX(),speakerPose.getY() - getPose().getY());
    SmartDashboard.putNumber("AprilTag targetAngle in degrees", targetAngle.getDegrees());
    drivetrain.setControl(
        fieldCentricFacingAngleRequest.withVelocityX(xVel.in(MetersPerSecond))
            .withVelocityY(yVel.in(MetersPerSecond))
            .withTargetDirection(targetAngle));
  }

  /**
   * Get robot centric speeds
   * @return robot centric chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return SwerveConstants.kinematics.toChassisSpeeds(drivetrain.getState().ModuleStates);
  }

  /**
   * Reset swerve odometry/pose tracking to 0,0
   */
  public void resetPose() {
    drivetrain.tareEverything();
  }

  /**
   * Gets calculated pose (from odometry with any added vision measurements)
   * 
   * @return
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
    drivetrain.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
  }

  /**
   * Applies vision measurement from NetworkTables data
   * 
   * @param poseArray double array storing the data of a Pose2d of format {x (meters), y (meters), rotation (radians)}
   * @param timestamp timestamp in microseconds
   */
  public void applyVisionMeasurement(double[] poseArray, long timestamp) {
    SmartDashboard.putNumberArray("Robot Pose", poseArray);
    // converts array of format {x (m), y (m), rotation (rad)} to Pose2d
    Pose2d pose = new Pose2d(poseArray[0], poseArray[1], new Rotation2d(poseArray[2]));
    double timestampSeconds = timestamp * UnitConstants.microsecondsToSeconds; // convert microseconds timestamp to seconds
    drivetrain.addVisionMeasurement(pose, timestampSeconds);
  }

  @Override
  public void periodic() {
  }
}
