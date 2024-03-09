// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistribution;

import static edu.wpi.first.units.Units.*;

/** Add your docs here. */
public class Constants {
    public static final String HighSpeedCANbusName = "Bobby";
    public static final String LowSpeedCANbusName = "rio";

    public static final int pigeon2Id = 0;
    public static final MountPoseConfigs pigeonMountConfigs = new MountPoseConfigs()
        .withMountPosePitch(0)
        .withMountPoseRoll(0)
        .withMountPoseYaw(180);

    public static final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    public static final double controllerDeadband = 0.225;

    public static final PowerDistribution pdh = new PowerDistribution();
    public static final double lowStartVoltage = 1.0;

    public static final class UnitConstants {
        public static final long secondsToMicroseconds = 1000000;
        public static final double microsecondsToSeconds = 1.0 / secondsToMicroseconds;
    }

    public static final class SwerveConstants {
        public static final Measure<Velocity<Distance>> defaultDriveSpeed = MetersPerSecond.of(3); // m/s
        public static final Measure<Velocity<Angle>> defaultRotSpeed = RadiansPerSecond.of(1.5 * Math.PI); // rad/s
        
        public static final Measure<Velocity<Distance>> fastDriveSpeed = MetersPerSecond.of(6); // m/s
        public static final Measure<Velocity<Angle>> fastRotSpeed = RadiansPerSecond.of(4 * Math.PI); // rad/s
        
        public static final Measure<Velocity<Distance>> slowDriveSpeed = MetersPerSecond.of(1); // m/s
        public static final Measure<Velocity<Angle>> slowRotSpeed = RadiansPerSecond.of(1 * Math.PI); // rad/s

        private static final Measure<Distance> swerveWidth = Inches.of(24); // width between centers of swerve modules
                                                                            // from left to right
        private static final Measure<Distance> swerveLength = Inches.of(24); // length between centers of swerve modules
                                                                             // from front to back
        private static final double driveMotorGearRatio = (8.14 / 1.0); // 8.14:1
        private static final double steerMotorGearRatio = (150.0 / 7 / 1.0); // 7:1
        private static final Measure<Distance> wheelRadius = Inches.of(1.97);
        private static final Measure<Current> slipCurrent = Amps.of(400);
        
        public static final PIDController headingPID = new PIDController(6.5,0.1,0.); // controls PID rotating to angle
        public static final Measure<Velocity<Angle>> minAngularVel = DegreesPerSecond.of(30);
        public static final Measure<Angle> rotationalPIDTolerance = Degrees.of(0.5);

        private static final Slot0Configs steerMotorGains = new Slot0Configs()
                .withKP(100.0) // output (V) per unit error in position (rotations)
                .withKI(150.0) // output (V) per unit integrated error (rotations*s)
                .withKD(60.0) // output (V) per unit of error derivative (rps)
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

        public static final Measure<Velocity<Distance>> velocityDeadband = defaultDriveSpeed.times(0.02);
        public static final Measure<Velocity<Angle>> rotationDeadband = defaultRotSpeed.times(0.01);

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
            private static final double encoderOffset = -0.446;
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
            private static final double encoderOffset = 0.148;
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
            private static final double encoderOffset = -0.493;
            private static final double xPos = -swerveLength.in(Meters) / 2; // to front
            private static final double yPos = swerveWidth.in(Meters) / 2; // to left
            private static final boolean invertedSteer = false;
            private static final boolean invertedDrive = false;

            public static final SwerveModuleConstants moduleConstants = constantsCreator.createModuleConstants(
                    steerId, driveId, encoderId, encoderOffset, xPos, yPos, invertedDrive)
                    .withSteerMotorInverted(invertedSteer);
        }

        private static final class BackRight {
            private static final int steerId = 20;
            private static final int driveId = 23;
            private static final int encoderId = 3;
            private static final double encoderOffset = -0.4035;
            private static final double xPos = -swerveLength.in(Meters) / 2; // to front
            private static final double yPos = -swerveWidth.in(Meters) / 2; // to left
            private static final boolean invertedSteer = true;
            private static final boolean invertedDrive = false;

            public static final SwerveModuleConstants moduleConstants = constantsCreator.createModuleConstants(
                    steerId, driveId, encoderId, encoderOffset, xPos, yPos, invertedDrive)
                    .withSteerMotorInverted(invertedSteer);
        }

        private static final SwerveDrivetrainConstants drivetrainConstants = new SwerveDrivetrainConstants()
                .withCANbusName(HighSpeedCANbusName).withPigeon2Id(pigeon2Id);

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

        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Coast;
    }

    public static final class IntakeConstants {
        public static final Measure<Velocity<Distance>> moveOverVelocity = MetersPerSecond.of(1.); // velocity to move over note for intake
        
        public static final TalonFX intakeFeedMotor = new TalonFX(24, LowSpeedCANbusName);
        public static final InvertedValue intakeFeederInverted = InvertedValue.Clockwise_Positive; // clockwise is in

        public static final TalonSRX shooterFeedMotor = new TalonSRX(3);
        public static final boolean shooterFeedInverted = false; // positive is in

        public static final double fastFeedRate = 0.75; // Out of 1, how fast rollers should be driven
        public static final double slowFeedRate = .4;

        public static final DigitalInput intakeLidarSensor = new DigitalInput(2);
        public static final DigitalInput loadedNoteLidarSensor = new DigitalInput(3);

        public static final Measure<Distance> noteAlignTolerance = Inches.of(4);
        public static final Measure<Time> alignedDelay = Seconds.of(0.3);
    }

    public static final class VisionConstants {
        public static final PhotonCamera aprilTagCam = new PhotonCamera("AprilTag Limelight");
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        
        public static final Transform3d robotToAprilTagCam = new Transform3d(
                        new Translation3d(Inches.of(-13.5), Inches.of(-3.25), Inches.of(7)),
                        new Rotation3d(0.0, Degrees.of(20).in(Radians), Degrees.of(180).in(Radians)));
        public static final Transform3d robotToNoteDetectionCam = new Transform3d(
                new Translation3d(SwerveConstants.swerveLength.times(0.5), Meters.zero(), Inches.of(15)),
                new Rotation3d(0.0, Degrees.of(-34).in(Radians), 0.0));
        
        public static final PoseStrategy poseStrategy = PoseStrategy.LOWEST_AMBIGUITY;
        public static final DoubleArrayTopic poseTopic = inst.getDoubleArrayTopic("/Vision/Estimated Pose");

        private static final NetworkTable colorVisionTable = inst.getTable("photonvision").getSubTable("Note Detection Limelight");

        public static final DoubleTopic noteYawTopic = colorVisionTable.getDoubleTopic("targetYaw");
        public static final BooleanTopic colorHasTargetsTopic = colorVisionTable.getBooleanTopic("hasTarget");
        public static final DoubleTopic notePitchTopic = colorVisionTable.getDoubleTopic("targetPitch");
        
        // (x, y, theta) in meters and radians. increase for less confidence. default is (0.9, 0.9, 0.9)
        public static final Matrix<N3, N1> stdDvsMatrix = VecBuilder.fill(2.0, 2.0, 2.0); 
    }

    public static final class ShooterConstants {
        public static final Measure<Velocity<Distance>> exitVelocity = InchesPerSecond.of(590);
        public static final Measure<Angle> farAngle = Degrees.of(27);
        public static final Measure<Angle> closeAngle = Degrees.of(57);
        public static final Measure<Angle> defaultAngle = Degrees.of(35);

        public static final Slot0Configs flywheelGains = new Slot0Configs()
                .withKP(0.0) // output (V) per unit error in position (rps)
                .withKI(0.0) // output (V) per unit integrated error (rotations)
                .withKD(0.0) // output (V) per unit of error derivative (rps/s)
                .withKS(0) // output (V) to overcome static friction
                .withKV(0.14) // output (V) per unit of velocity (rps)
                .withKA(1.0); // output (V) per unit of acceleration (rps/s)

        public static final TalonFX topFlywheel = new TalonFX(32, HighSpeedCANbusName);
        public static final InvertedValue topFlywheelInverted = InvertedValue.Clockwise_Positive; // cw is shooting
        public static final TalonFX bottomFlywheel = new TalonFX(31, HighSpeedCANbusName);
        public static final InvertedValue bottomFlywheelInverted = InvertedValue.CounterClockwise_Positive; // ccw is shooting

        public static final Measure<Velocity<Angle>> topSpeed = RPM.of(6000);
        public static final Measure<Velocity<Angle>> bottomSpeed = topSpeed;
        public static final Measure<Velocity<Angle>> readySpeedTolerance = RPM.of(800);
        public static final Measure<Velocity<Angle>> closeSpeedTolerance = RPM.of(2000);
        public static final Measure<Velocity<Velocity<Angle>>> flywheelAccel = RPM.per(Second).of(6000); 

        public static final Measure<Velocity<Velocity<Distance>>> gravity = MetersPerSecondPerSecond.of(9.81);

        // Linkage characterization

        // Horizontal distance from shooter hinge to bar pivot
        public static final Measure<Distance> horizontalDistanceToPivot = Inches.of(4);
        // Height of bar pivot compared to shooter hinge
        public static final Measure<Distance> pivotHeight = Inches.of(0.);
        public static final Measure<Distance> topBarLength = Inches.of(2);
        public static final Measure<Distance> bottomBarLength = Inches.of(6.2);
        public static final Measure<Distance> motorMountHeight = Inches.of(2.125); // height of motor above shooter
        public static final Measure<Distance> motorDistance = Inches.of(7.38); // distance of motor along shooter

        public static final TalonFX heightMotor = new TalonFX(26);
        public static final double maxOutput = 1.0; // duty cycle output max
        public static final double arbitraryFeedforward = 0.015; // duty cycle arbitrary feed forward to account for gravity
        public static final double heightMotorGearRatio = 100.0/1; // 100:1
        public static final Measure<Angle> lowMotorAngle = Rotations.of(0.419);
        public static final Measure<Angle> highMotorAngle = Rotations.of(0.236);
        
        // height motor configuration
        private static final MotorOutputConfigs heightMotorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        private static final CurrentLimitsConfigs heightCurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(20)
                .withSupplyCurrentThreshold(40)
                .withSupplyTimeThreshold(0.1);
        
        public static final TalonFXConfiguration heightMotorConfigs = new TalonFXConfiguration()
                .withMotorOutput(heightMotorOutputConfigs)
                .withCurrentLimits(heightCurrentLimits);
        
        // height motor PID
        public static final PIDController heightPID = new PIDController(3.0, 0.2, 0);
        public static final Measure<Angle> heightTolerance = Degrees.of(1.);

        public static final DutyCycleEncoder heightMotorEncoder = new DutyCycleEncoder(0); // DIO port 0
        public static final int minPulseMicroseconds = 1;
        public static final int maxPulseMicroseconds = 1024;
        public static final double encoderValueAt90Deg = 0.495; // encoder value in rotations

        // manual height control
        public static final Measure<Angle> manualSpeed = Degrees.of(2.5); // how fast to raise/lower manually

        // alignment
        public static final Measure<Distance> readyAlignTolerance = Inches.of(12);
    }

    public static final class FieldConstants {
        public static final Translation3d redSpeakerPos = new Translation3d(Inches.of(652.73), Inches.of(218.42),
                Inches.of(80.5));
        public static final Translation3d blueSpeakerPos = new Translation3d(Inches.of(-1.50), Inches.of(218.42),
                Inches.of(80.5));
    }

    public static final class TrapConstants {
        public static final CANSparkMax heightControlMotor = new CANSparkMax(15, MotorType.kBrushless);
        public static final boolean heightMotorInverted = true; // positive is up
        
        public static final TalonSRX rollerMotor = new TalonSRX(12);
        public static final boolean rollerInverted = false; // positive is through

        public static final Servo linearActuator = new Servo(2); // flips mechanism down
        public static final int maxMicroseconds = 2000;
        public static final int centerMicroseconds = 1500;
        public static final int minMicroseconds = 1000;
        public static final double ampActuatorPosition = 0.3;

        public static final Measure<Distance> homePosition = Inches.of(0);
        public static final Measure<Distance> ampPosition = Inches.of(14); // height to stop at for amp, measured from lowest position
        public static final Measure<Distance> trapPosition = Inches.of(17.5); // height to stop at for trap, measured from lowest pos
        public static final Measure<Distance> sourcePosition = Inches.of(5); // height to stop at for trap, measured from lowest pos
        public static final Measure<Distance> maxHeight = Inches.of(20);

        public static final Encoder heightEncoder = new Encoder(7, 6); // encoder for vertical movement
        private static final int pulsesPerRev = 2048; // number full encoder cycles per revolution
        public static final Measure<Distance> distancePerPulse = Inches.of(9.0/pulsesPerRev); // distance per encoder pulse (9in per revolution)
        public static final Measure<Distance> heightTolerance = Inches.of(0.25);
        public static final PIDController heightPIDController = new PIDController(0.3, 0.0, 0.0);

        public static final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        public static final Color colorToMatch = new Color("#20617C"); // match blue tape

        public static final double intakeSpeed = 0.9; // out of 1, how fast to feed note in (from source)
        public static final double expelSpeed = 1; // out of 1, how fast to expel note (pretty much never used)
        public static final double raiseSpeed = 0.25; // out of 1, max speed to raise
        public static final double lowerSpeed = 0.1; // out of 1, speed to home to zero

        public static final DigitalInput noteDetectionLidarSensor = new DigitalInput(1);
        public static final Measure<Time> intakeDelay = Seconds.of(0.15); // time to wait after note detected by lidar
    }

    public static final class ClimberConstants {
        public static final TalonFX climberMotor = new TalonFX(4, LowSpeedCANbusName);
        public static final InvertedValue climberInverted = InvertedValue.CounterClockwise_Positive; // CCW climbs
        public static final double climbSpeed =         1;

        public static final DigitalInput limitLidarSensor = new DigitalInput(5);
    }

    public static final class AutoConstants {
        public static final HolonomicPathFollowerConfig holonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(0.5, 0.0, 0.0), // translational PID
                new PIDConstants(0.5, 0.0, 0.0), // rotational PID
                SwerveConstants.defaultDriveSpeed.in(MetersPerSecond), // max drive speed
                // radius of drivetrain (distance from center to furthest module)
                new Translation2d(SwerveConstants.swerveLength.divide(2), SwerveConstants.swerveWidth.divide(2)).getNorm(),
                new ReplanningConfig() // default replanning config
        );
    }

    public static final class LoggingConstants { 
        public static final NetworkTable generalTable = inst.getTable("General");
        public static final NetworkTable shooterTable = inst.getTable("Shooter");
        public static final NetworkTable trapTable = inst.getTable("Trap");
        public static final NetworkTable swerveTable = inst.getTable("Swerve");
        public static final NetworkTable climberTable = inst.getTable("Climber");
        public static final NetworkTable intakeTable = inst.getTable("Intake");

        // Trap Logs
        public static final DoublePublisher trapHeightPublisher = trapTable.getDoubleTopic("TrapHeight").publish();
        public static final DoublePublisher actuatorPublisher = trapTable.getDoubleTopic("ActuatorExtended").publish(); 
        public static final DoublePublisher trapMechanismSpeedPublisher = trapTable.getDoubleTopic("Trap Mechanism Speed").publish();
        public static final DoublePublisher trapMechanismSetpointPublisher = trapTable.getDoubleTopic("Trap Mechanism Setpoint").publish();
        public static final BooleanPublisher colorSensorPublisher = trapTable.getBooleanTopic("Color Sensor").publish(); 
        public static final BooleanPublisher trapLIDARPublisher = trapTable.getBooleanTopic("Trap Intake LIDAR").publish();
        
        // Climber Logs
        public static final BooleanPublisher climberDownPublisher = climberTable.getBooleanTopic("Climber Down").publish();
        public static final DoublePublisher climberSpeedPublisher = climberTable.getDoubleTopic("Climber Speed").publish();

        // Shooter Logs
        public static final DoublePublisher shooterSpeedPublisher = shooterTable.getDoubleTopic("Shooter Speed").publish();
        public static final DoublePublisher shooterSpeedRequestPublisher = shooterTable.getDoubleTopic("Shooter Speed Request").publish();
        public static final BooleanPublisher shooterAtSpeedPublisher = shooterTable.getBooleanTopic("Shooter At Speed").publish();
        public static final BooleanPublisher shooterNearSpeedPublisher = shooterTable.getBooleanTopic("Shooter Near Speed").publish();
        public static final DoublePublisher shooterAnglePublisher = shooterTable.getDoubleTopic("Shooter Angle").publish();
        public static final DoublePublisher shooterAngleRequestPublisher = shooterTable.getDoubleTopic("Shooter Angle request").publish();
        public static final BooleanPublisher alignedToSpeakerPublisher = shooterTable.getBooleanTopic("Aligned To Speaker").publish();

        // Intake Logs
        public static final BooleanPublisher intakeLIDARPublisher = intakeTable.getBooleanTopic("Intake LIDAR").publish();
        public static final BooleanPublisher noteLoadedPublisher = intakeTable.getBooleanTopic("Note Loaded").publish();
        public static final DoublePublisher intakeSpeedPublisher = intakeTable.getDoubleTopic("Intake Speed").publish();
        public static final DoublePublisher feederSpeedPublisher = intakeTable.getDoubleTopic("Feeder Speed").publish();

        // Swerve Logs
        public static final StructArrayPublisher<SwerveModuleState> moduleStatesPublisher = swerveTable
                .getStructArrayTopic("Swerve Module States", SwerveModuleState.struct).publish();
        public static final StructArrayPublisher<SwerveModuleState> desiredModuleStatesPublisher = swerveTable
                .getStructArrayTopic("Target Swerve Module States", SwerveModuleState.struct).publish();
        public static final DoublePublisher rotationPublisher = swerveTable.getDoubleTopic("Rotation").publish();
        public static final DoubleArrayPublisher chassisSpeedsPublisher = swerveTable.getDoubleArrayTopic("Chassis Speeds").publish();
    }

    public static final class LightConstants
    {
        // green and blue are flipped
        public static final Color8Bit RED = new Color8Bit(255,0,0);
        public static final Color8Bit ORANGE = new Color8Bit(255, 0, 40);
        public static final Color8Bit YELLOW = new Color8Bit(255, 0, 70);
        public static final Color8Bit GREEN = new Color8Bit(0,0,255);
        public static final Color8Bit DARK_GREEN = new Color8Bit(0,0,0);
        public static final Color8Bit BLUE = new Color8Bit(0,255,0);
        public static final Color8Bit PURPLE = new Color8Bit(91, 104, 0);
        public static final Color8Bit PINK = new Color8Bit(244,235,0);
        public static final Color8Bit BLANK = new Color8Bit(0, 0, 0);

        public static final int numLEDGroups = 12;

        public static final double rainbowSpeed = 1;

        public static final Color8Bit[] rainbow = new Color8Bit[] {
                RED, ORANGE, YELLOW, 
                GREEN, BLUE, PURPLE, 
                RED, ORANGE, YELLOW,
                GREEN, BLUE, PURPLE
        };

        public static final double trailSpeed = 4;

        public static final Color8Bit[] greenTrails = new Color8Bit[] {
                GREEN, DARK_GREEN, BLANK,
                BLANK, DARK_GREEN, GREEN,
                GREEN, DARK_GREEN, BLANK,
                BLANK, DARK_GREEN, GREEN
        };

        public static final int[] forwardShiftIndices = new int[] {
                1, 2, 0,
                5, 3, 4,
                7, 8, 6,
                11, 9, 10
        };
    }
}
