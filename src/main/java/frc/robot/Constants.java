// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/** Add your docs here. */
public class Constants {
    public static final String CANbusName = "Bobby";
    public static final int pigeon2Id = 0;

    public static final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    public static final class UnitConstants {
        public static final long secondsToMicroseconds = 1000000;
        public static final double microsecondsToSeconds = 1.0 / secondsToMicroseconds;
    }

    public static final class SwerveConstants {
        public static final Measure<Velocity<Distance>> maxDriveSpeed = MetersPerSecond.of(4); // m/s
        public static final Measure<Velocity<Angle>> maxRotSpeed = RadiansPerSecond.of(1.5 * Math.PI); // rad/s

        private static final Measure<Distance> swerveWidth = Inches.of(24); // width between centers of swerve modules
                                                                            // from left to right
        private static final Measure<Distance> swerveLength = Inches.of(24); // length between centers of swerve modules
                                                                             // from front to back
        private static final double driveMotorGearRatio = (8.14 / 1.0); // 8.14:1
        private static final double steerMotorGearRatio = (150.0 / 7 / 1.0); // 7:1
        private static final Measure<Distance> wheelRadius = Inches.of(1.97);
        private static final Measure<Current> slipCurrent = Amps.of(400);
        
        public static final PhoenixPIDController headingPID = new PhoenixPIDController(3.0,0,0); // controls PID rotating to angle
        public static final Measure<Angle> speakerRotationalTolerance = Degrees.of(1);
        public static final PIDController noteAlignPID = new PIDController(2.5, 0, 0);
        public static final Measure<Angle> noteRotationalTolerance = Degrees.of(5);

        private static final Slot0Configs steerMotorGains = new Slot0Configs()
                .withKP(50.0) // output (V) per unit error in position (rotations)
                .withKI(0.0) // output (V) per unit integrated error (rotations*s)
                .withKD(50.0) // output (V) per unit of error derivative (rps)
                .withKS(0) // output (V) to overcome static friction
                .withKV(2.5); // output (V) per unit of velocity (rps)
        private static final Slot0Configs driveMotorGains = new Slot0Configs()
                .withKP(1.0) // output (V) per unit error in position (rps)
                .withKI(0.0) // output (V) per unit integrated error (rotations)
                .withKD(0.0) // output (V) per unit of error derivative (rps/s)
                .withKS(0) // output (V) to overcome static friction
                .withKV(0.123) // output (V) per unit of velocity (rps)
                .withKA(0); // output (V) per unit of acceleration (rps/s)

        private static final ClosedLoopOutputType steerMotorClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;
        private static final ClosedLoopOutputType driveMotorClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;
        private static final Measure<Velocity<Distance>> speedAt12Volts = FeetPerSecond.of(13.0);
        private static final SteerFeedbackType feedbackSource = SteerFeedbackType.FusedCANcoder;
        private static final double couplingGearRatio = 50.0 / 14.0;

        public static final DriveRequestType driveRequestType = DriveRequestType.Velocity;
        public static final SteerRequestType steerRequestType = SteerRequestType.MotionMagicExpo;

        public static final Measure<Velocity<Distance>> velocityDeadband = maxDriveSpeed.times(0.02);
        public static final Measure<Velocity<Angle>> rotationDeadband = maxRotSpeed.times(0.02);

        public static final double controllerDeadband = 0.1;


        private static final SwerveModuleConstantsFactory constantsCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(driveMotorGearRatio)
                .withSteerMotorGearRatio(steerMotorGearRatio)
                .withWheelRadius(wheelRadius.in(Inches))
                .withSlipCurrent(slipCurrent.in(Amps))
                .withSteerMotorGains(steerMotorGains)
                .withDriveMotorGains(driveMotorGains)
                .withSteerMotorClosedLoopOutput(steerMotorClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(driveMotorClosedLoopOutput)
                .withSpeedAt12VoltsMps(speedAt12Volts.in(MetersPerSecond))
                .withFeedbackSource(feedbackSource)
                .withCouplingGearRatio(couplingGearRatio);

        private static final class FrontLeft {
            private static final int steerId = 7;
            private static final int driveId = 22;
            private static final int encoderId = 2;
            private static final double encoderOffset = -0.456;
            private static final double xPos = swerveLength.in(Meters) / 2; // to front
            private static final double yPos = swerveWidth.in(Meters) / 2; // to left
            private static final boolean invertedSteer = true;
            private static final boolean invertedDrive = true;

            public static final SwerveModuleConstants moduleConstants = constantsCreator.createModuleConstants(
                    steerId, driveId, encoderId, encoderOffset, xPos, yPos, invertedDrive)
                    .withSteerMotorInverted(invertedSteer);
        }

        private static final class FrontRight {
            private static final int steerId = 1;
            private static final int driveId = 6;
            private static final int encoderId = 0;
            private static final double encoderOffset = 0.1455;
            private static final double xPos = swerveLength.in(Meters) / 2; // to front
            private static final double yPos = -swerveWidth.in(Meters) / 2; // to left
            private static final boolean invertedSteer = false;
            private static final boolean invertedDrive = true;

            public static final SwerveModuleConstants moduleConstants = constantsCreator.createModuleConstants(
                    steerId, driveId, encoderId, encoderOffset, xPos, yPos, invertedDrive)
                    .withSteerMotorInverted(invertedSteer);
        }

        private static final class BackLeft {
            private static final int steerId = 21;
            private static final int driveId = 0;
            private static final int encoderId = 1;
            private static final double encoderOffset = -0.485;
            private static final double xPos = -swerveLength.in(Meters) / 2; // to front
            private static final double yPos = swerveWidth.in(Meters) / 2; // to left
            private static final boolean invertedSteer = false;
            private static final boolean invertedDrive = false;

            public static final SwerveModuleConstants moduleConstants = constantsCreator.createModuleConstants(
                    steerId, driveId, encoderId, encoderOffset, xPos, yPos, invertedDrive)
                    .withSteerMotorInverted(invertedSteer);
        }

        private static final class BackRight {
            private static final int steerId = 5;
            private static final int driveId = 23;
            private static final int encoderId = 3;
            private static final double encoderOffset = -0.41;
            private static final double xPos = -swerveLength.in(Meters) / 2; // to front
            private static final double yPos = -swerveWidth.in(Meters) / 2; // to left
            private static final boolean invertedSteer = true;
            private static final boolean invertedDrive = false;

            public static final SwerveModuleConstants moduleConstants = constantsCreator.createModuleConstants(
                    steerId, driveId, encoderId, encoderOffset, xPos, yPos, invertedDrive)
                    .withSteerMotorInverted(invertedSteer);
        }

        private static final SwerveDrivetrainConstants drivetrainConstants = new SwerveDrivetrainConstants()
                .withCANbusName(CANbusName).withPigeon2Id(pigeon2Id);

        public static final SwerveDrivetrain drivetrain = new SwerveDrivetrain(
                drivetrainConstants,
                FrontLeft.moduleConstants,
                FrontRight.moduleConstants,
                BackLeft.moduleConstants,
                BackRight.moduleConstants);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(FrontLeft.xPos, FrontLeft.yPos),
                new Translation2d(FrontRight.xPos, FrontRight.yPos),
                new Translation2d(BackLeft.xPos, BackLeft.yPos),
                new Translation2d(BackRight.xPos, BackRight.yPos));

        public static final CurrentLimitsConfigs angleCurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(20)
                .withSupplyCurrentThreshold(40)
                .withSupplyTimeThreshold(0.1);

        public static final CurrentLimitsConfigs driveCurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentThreshold(40)
                .withSupplyTimeThreshold(0.1);

        public static final OpenLoopRampsConfigs openLoopRampsConfig = new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(0.25)
                .withVoltageOpenLoopRampPeriod(0.25)
                .withTorqueOpenLoopRampPeriod(0.25);

        public static final ClosedLoopRampsConfigs closedLoopRampsConfig = new ClosedLoopRampsConfigs()
                .withDutyCycleClosedLoopRampPeriod(0.0)
                .withVoltageClosedLoopRampPeriod(0.0)
                .withTorqueClosedLoopRampPeriod(0.0);

        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
        
        public static final Translation3d redSpeakerPos = new Translation3d(Inches.of(652.73),Inches.of(196.17),Inches.of(57.13));
        public static final Translation3d blueSpeakerPos = new Translation3d(Inches.of(-1.50),Inches.of(218.42),Inches.of(57.13));

    }

    public static final class IntakeConstants {
        public static final int frontLidarSensorChannel = 0;
        public static final int loadLiderSensorChannel = 0;
        public static final Measure<Velocity<Distance>> moveOverVelocity = MetersPerSecond.of(1.); // velocity to move over note for intake
    }

    public static final class VisionConstants {
        public static final PhotonCamera aprilTagCam = new PhotonCamera("Camera_Module_v1");
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        
        public static final Transform3d robotToAprilTagCam = new Transform3d(
                        new Translation3d(SwerveConstants.swerveLength, Meters.zero(), Meters.zero()),
                        new Rotation3d(0.0, 0.0, 0.0));
        
        public static final PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
        public static final DoubleArrayTopic poseTopic = inst.getDoubleArrayTopic("/Vision/Estimated Pose");

        private static final NetworkTable colorVisionTable = inst.getTable("photonvision").getSubTable("Camera_Module_v1");

        public static final DoubleTopic noteYawTopic = colorVisionTable.getDoubleTopic("targetYaw");
        public static final BooleanTopic colorHasTargetsTopic = colorVisionTable.getBooleanTopic("hasTarget");
    }

    public static final class AutoConstants {
        public static final HolonomicPathFollowerConfig holonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(0.5, 0.0, 0.0), // translational PID
            new PIDConstants(0.5, 0.0, 0.0), // rotational PID
            SwerveConstants.maxDriveSpeed.in(MetersPerSecond), // max drive speed
            new Translation2d(SwerveConstants.swerveLength, SwerveConstants.swerveWidth).getNorm(), // radius of drivetrain (distance from center to furthest module)
            new ReplanningConfig() // default replanning config
        );
    }
}
