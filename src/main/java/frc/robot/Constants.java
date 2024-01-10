// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

/** Add your docs here. */
public class Constants {
    public static final String CANbusName = "Bobby";
    public static final int pigeon2Id;

    public static final class Swerve {
        // TODO: All the constants
        private static final double driveMotorGearRatio;
        private static final double steerMotorGearRatio;
        private static final double wheelRadius;
        private static final double slipCurrent;
        private static final Slot0Configs steerMotorGains;
        private static final Slot0Configs driveMotorGains;
        private static final ClosedLoopOutputType steerMotorClosedLoopOutput;
        private static final ClosedLoopOutputType driveMotorClosedLoopOutput;
        private static final double speedAt12VoltsMps;
        private static final double steerInertia;
        private static final double driveInertia;
        private static final SteerFeedbackType feedbackSource;
        private static final double couplingGearRatio;
        private static final boolean steerMotorInverted;

        public static final double velocityDeadband = 0.1; // in m/s
        public static final double rotationDeadband = 0.1; // in radians

        private static final SwerveModuleConstantsFactory constantsCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(driveMotorGearRatio)
            .withSteerMotorGearRatio(steerMotorGearRatio)
            .withWheelRadius(wheelRadius)
            .withSlipCurrent(slipCurrent)
            .withSteerMotorGains(steerMotorGains)
            .withDriveMotorGains(driveMotorGains)
            .withSteerMotorClosedLoopOutput(steerMotorClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveMotorClosedLoopOutput)
            .withSpeedAt12VoltsMps(speedAt12VoltsMps)
            .withSteerInertia(steerInertia)
            .withDriveInertia(driveInertia)
            .withFeedbackSource(feedbackSource)
            .withCouplingGearRatio(couplingGearRatio)
            .withSteerMotorInverted(steerMotorInverted);

        private static final class FrontLeft {
            private static final double steerId;
            private static final double driveId;
            private static final double encoderId;
            private static final double encoderOffset;
            private static final double xPos;
            private static final double yPos;
            private static final boolean invertedDrive = false;

            public static final SwerveModuleConstants moduleConstants = constantsCreator.createModuleConstants(
            steerId, driveId, encoderId, encoderOffset, xPos, yPos, invertedDrive);
        }

        private static final class FrontRight {
            private static final int steerId;
            private static final int driveId;
            private static final int encoderId;
            private static final double encoderOffset;
            private static final double xPos;
            private static final double yPos;
            private static final boolean invertedDrive = false;

            public static final SwerveModuleConstants moduleConstants = constantsCreator.createModuleConstants(
            steerId, driveId, encoderId, encoderOffset, xPos, yPos, invertedDrive);
        }

        private static final class BackLeft {
            private static final int steerId;
            private static final int driveId;
            private static final int encoderId;
            private static final double encoderOffset;
            private static final double xPos;
            private static final double yPos;
            private static final boolean invertedDrive = false;

            public static final SwerveModuleConstants moduleConstants = constantsCreator.createModuleConstants(
            steerId, driveId, encoderId, encoderOffset, xPos, yPos, invertedDrive);
        }

        private static final class BackRight {
            private static final int steerId;
            private static final int driveId;
            private static final int encoderId;
            private static final double encoderOffset;
            private static final double xPos;
            private static final double yPos;
            private static final boolean invertedDrive = false;

            public static final SwerveModuleConstants moduleConstants = constantsCreator.createModuleConstants(
            steerId, driveId, encoderId, encoderOffset, xPos, yPos, invertedDrive);
        }
        
        private static final SwerveDrivetrainConstants drivetrainConstants = new SwerveDrivetrainConstants()
            .withCANbusName(CANbusName).withPigeon2Id(pigeon2Id);

        public static final SwerveDrivetrain drivetrain = new SwerveDrivetrain(
            drivetrainConstants, 
            FrontLeft.moduleConstants, 
            FrontRight.moduleConstants, 
            BackLeft.moduleConstants, 
            BackRight.moduleConstants
        );
    }
}
