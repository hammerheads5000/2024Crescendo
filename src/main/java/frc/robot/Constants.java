// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;

/** Add your docs here. */
public class Constants {
    public static final String CANbusName = "Bobby";
    public static final int pigeon2Id = 0;

    public static final class SwerveConstants {
        // TODO: All the constants
        public static Measure<Velocity<Distance>> maxDriveSpeed = MetersPerSecond.of(6); // m/s
        public static Measure<Velocity<Angle>> maxRotSpeed = RadiansPerSecond.of(1.5*Math.PI); // rad/s

        private static final Measure<Distance> swerveWidth = Inches.of(24); // width between centers of swerve modules from left to right
        private static final Measure<Distance> swerveLength = Inches.of(24); // length between centers of swerve modules from front to back
        private static final double driveMotorGearRatio = (8.14 / 1.0); // 8.14:1
        private static final double steerMotorGearRatio = (150.0 / 7 / 1.0); // 7:1
        private static final Measure<Distance> wheelRadius = Inches.of(1.97);
        private static final Measure<Current> slipCurrent = Amps.of(400);

        private static final Slot0Configs steerMotorGains = new Slot0Configs()
                                                            .withKP(100) // output (V) per unit error in position (rotations)
                                                            .withKI(0.0) // output (V) per unit integrated error (rotations*s)
                                                            .withKD(0.2) // output (V) per unit of error derivative (rps)
                                                            .withKS(0) // output (V) to overcome static friction
                                                            .withKV(0.12); // output (V) per unit of velocity (rps)
        private static final Slot0Configs driveMotorGains = new Slot0Configs()
                                                            .withKP(1) // output (V) per unit error in position (rps)
                                                            .withKI(0.0) // output (V) per unit integrated error (rotations)
                                                            .withKD(0.0) // output (V) per unit of error derivative (rps/s)
                                                            .withKS(0) // output (V) to overcome static friction
                                                            .withKV(.14) // output (V) per unit of velocity (rps)
                                                            .withKA(0); // output (V) per unit of acceleration (rps/s)

        private static final ClosedLoopOutputType steerMotorClosedLoopOutput = ClosedLoopOutputType.Voltage;
        private static final ClosedLoopOutputType driveMotorClosedLoopOutput = ClosedLoopOutputType.Voltage;
        private static final Measure<Velocity<Distance>> speedAt12Volts = FeetPerSecond.of(13.0);
        private static final SteerFeedbackType feedbackSource = SteerFeedbackType.FusedCANcoder;
        private static final double couplingGearRatio = 50.0/14.0;

        public static final Measure<Velocity<Distance>> velocityDeadband = maxDriveSpeed.times(0.1); // in m/s
        public static final Measure<Velocity<Angle>> rotationDeadband = maxRotSpeed.times(0.1); // in rad/s

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
            .withCouplingGearRatio(couplingGearRatio)
            ;

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
                steerId, driveId, encoderId, encoderOffset, xPos, yPos, invertedDrive
            ).withSteerMotorInverted(invertedSteer);
        }

        private static final class FrontRight {
            private static final int steerId = 1;
            private static final int driveId = 6;
            private static final int encoderId = 0;
            private static final double encoderOffset = 0.1455;
            private static final double xPos = swerveLength.in(Meters) / 2; // to front
            private static final double yPos = -swerveWidth.in(Meters) / 2; // to left
            private static final boolean invertedSteer = false;
            private static final boolean invertedDrive = false;

            public static final SwerveModuleConstants moduleConstants = constantsCreator.createModuleConstants(
                steerId, driveId, encoderId, encoderOffset, xPos, yPos, invertedDrive
            ).withSteerMotorInverted(invertedSteer);
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
                steerId, driveId, encoderId, encoderOffset, xPos, yPos, invertedDrive
            ).withSteerMotorInverted(invertedSteer);
        }

        private static final class BackRight {
            private static final int steerId = 5;
            private static final int driveId = 23;
            private static final int encoderId = 3;
            private static final double encoderOffset = -0.41;
            private static final double xPos = -swerveLength.in(Meters) / 2; // to front
            private static final double yPos = -swerveWidth.in(Meters) / 2; // to left
            private static final boolean invertedSteer = true;
            private static final boolean invertedDrive = true;

            public static final SwerveModuleConstants moduleConstants = constantsCreator.createModuleConstants(
                steerId, driveId, encoderId, encoderOffset, xPos, yPos, invertedDrive
            ).withSteerMotorInverted(invertedSteer);
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

        public static final CurrentLimitsConfigs angleCurrentLimits = new CurrentLimitsConfigs()
                                            .withSupplyCurrentLimit(20)
                                            .withSupplyCurrentThreshold(40)
                                            .withSupplyTimeThreshold(0.1);
        
        public static final CurrentLimitsConfigs driveCurrentLimits = new CurrentLimitsConfigs()
                                            .withSupplyCurrentLimit(40)
                                            .withSupplyCurrentThreshold(40)
                                            .withSupplyTimeThreshold(0.1);
        
        public static final OpenLoopRampsConfigs openLoopRampsConfig = new OpenLoopRampsConfigs()
                            .withDutyCycleOpenLoopRampPeriod(0.25);
        
        public static final ClosedLoopRampsConfigs closedLoopRampsConfig = new ClosedLoopRampsConfigs()
                            .withDutyCycleClosedLoopRampPeriod(0.0);

        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    }
}
