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
  private PhotonPoseEstimator poseEstimatorFront;
  private PhotonPoseEstimator poseEstimatorBack;

  private DoubleArrayPublisher publisherFront;
  private DoubleArrayPublisher publisherBack;

  private boolean hasTarget = false;

  /** Creates a new AprilTagSubsystem. */
  public AprilTagSubsystem() {
    fieldLayout = VisionConstants.aprilTagFieldLayout;
    poseEstimatorFront = new PhotonPoseEstimator(fieldLayout, VisionConstants.poseStrategy,
        VisionConstants.aprilTagCamFront, VisionConstants.robotToAprilTagCamFront);
    poseEstimatorBack = new PhotonPoseEstimator(fieldLayout, VisionConstants.poseStrategy,
        VisionConstants.aprilTagCamBack, VisionConstants.robotToAprilTagCamBack);

    publisherFront = VisionConstants.poseTopicFront.publish();
    publisherBack = VisionConstants.poseTopicBack.publish();
  }

  public void update() {
    boolean frontTarget = updatePublisher(poseEstimatorFront, publisherFront);
    boolean backTarget = updatePublisher(poseEstimatorBack, publisherBack);

    hasTarget = frontTarget || backTarget;
  }

  private boolean updatePublisher(PhotonPoseEstimator estimator, DoubleArrayPublisher publisher) {
    Optional<EstimatedRobotPose> optionalPose = estimator.update();

    if (optionalPose.isEmpty()) {
      return false;
    }

    EstimatedRobotPose estimatedRobotPose = optionalPose.get();
    Pose2d pose = estimatedRobotPose.estimatedPose.toPose2d();
    
    // represents the pose as a double array with {x (meters), y (meters), rotation (radians)}
    double[] poseArray = new double[] {
      pose.getX(), pose.getY(),
      pose.getRotation().getRadians()
    };
    long timeMicroseconds = (long)(estimatedRobotPose.timestampSeconds * UnitConstants.secondsToMicroseconds);

    publisher.set(poseArray, timeMicroseconds);

    return true;
  }
  

  public boolean hasAprilTag() {
    return hasTarget;
  }

  @Override
  public void periodic() {
    update();
  }
}
