// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase {
  private SwerveDrivetrain drivetrain;
  private SwerveRequest.FieldCentric fieldCentricRequest;
  private SwerveRequest.RobotCentric robotCentricRequest;
  private SwerveRequest.ApplyChassisSpeeds applyChassisSpeedsRequest;

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
   * Applies estimated pose from AprilTag vision measurement
   * 
   * @param pose Pose of robot found with {@link org.photonvision.PhotonUtils#estimateFieldToRobotAprilTag(Transform3d, Pose3d, Transform3d) PhotonUtils.estimateFieldToRobotAprilTag}
   * @param timestamp Timestamp since FPGA startup found with {@link org.photonvision.targeting.PhotonPipelineResult#getTimestampSeconds() PhotonPipelineResult.getTimestampSeconds}
   */
  public void applyVisionMeasurement(Pose3d pose, Measure<Time> timestamp) {
    drivetrain.addVisionMeasurement(getPose(), timestamp.in(Seconds));
  }

  @Override
  public void periodic() {
  }
}
