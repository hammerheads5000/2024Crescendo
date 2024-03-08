// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UnitConstants;
import frc.robot.Constants.VisionConstants;

public class AprilTagSubsystem extends SubsystemBase {
  private AprilTagFieldLayout fieldLayout;
  private PhotonPoseEstimator poseEstimator;

  private DoubleArrayPublisher publisher;

  private boolean hasTarget = false;

  /** Creates a new AprilTagSubsystem. */
  public AprilTagSubsystem() {
    fieldLayout = VisionConstants.aprilTagFieldLayout;
    poseEstimator = new PhotonPoseEstimator(fieldLayout, VisionConstants.poseStrategy,
        VisionConstants.aprilTagCam, VisionConstants.robotToAprilTagCam);

    publisher = VisionConstants.poseTopic.publish();
  }

  public void update() {
    Optional<EstimatedRobotPose> optionalPose = poseEstimator.update();

    if (optionalPose.isEmpty()) {
      hasTarget = false;
      return;
    }

    hasTarget = true;
    EstimatedRobotPose estimatedRobotPose = optionalPose.get();
    Pose2d pose = estimatedRobotPose.estimatedPose.toPose2d();
    
    // represents the pose as a double array with {x (meters), y (meters), rotation (radians)}
    double[] poseArray = new double[] {
      pose.getX(), pose.getY(),
      pose.getRotation().getRadians()
    };
    long timeMicroseconds = (long)(estimatedRobotPose.timestampSeconds * UnitConstants.secondsToMicroseconds);

    publisher.set(poseArray, timeMicroseconds);
  }

  public boolean hasAprilTag() {
    return hasTarget;
  }

  @Override
  public void periodic() {
    update();
  }
}
