// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.ColoredTargetAutomatedSwerve;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable Photon = inst.getTable("photonvision");
  NetworkTable ColorVision = Photon.getSubTable("Camera_Module_v1");
  DoubleTopic nTop = ColorVision.getDoubleTopic("pipelineIndexState");
  DoubleTopic TargetYawTop = ColorVision.getDoubleTopic("targetYaw");
  DoubleArrayTopic ColorTargetPoseTopic = ColorVision.getDoubleArrayTopic("targetPose");
  BooleanTopic ColorHasTargetsTopic = ColorVision.getBooleanTopic("hasTarget");

  private Swerve swerve = new Swerve();
  private CommandXboxController controller = new CommandXboxController(0);
  private TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, controller);

  private AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem();
  private ColoredTargetAutomatedSwerve CTAS = new ColoredTargetAutomatedSwerve(ColorTargetPoseTopic,TargetYawTop, ColorHasTargetsTopic, swerve, controller);

  private Trigger zeroTrigger = controller.y();

  public RobotContainer() {
    swerve.setDefaultCommand(teleopSwerve);
    swerve.resetPose();
    configureBindings();
  }

  private void configureBindings() {
    zeroTrigger.onTrue(new InstantCommand(() -> swerve.resetPose()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
