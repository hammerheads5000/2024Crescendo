// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.FaceSpeaker;
import frc.robot.commands.IntakeCommandGroup;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private CommandXboxController controller = new CommandXboxController(0);

  // subsystems
  private Swerve swerve = new Swerve();
  private AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem();
  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  
  // commands
  private FaceSpeaker faceAngle = new FaceSpeaker(swerve, controller);
  private TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, controller);
  private IntakeCommandGroup intakeCommandGroup = new IntakeCommandGroup(swerve, intakeSubsystem);
  
  // triggers
  private Trigger zeroTrigger = controller.y();
  private Trigger faceAngleTrigger = controller.leftBumper();
  private Trigger targetTrigger = controller.rightBumper();
  

  public RobotContainer() {
    swerve.setDefaultCommand(teleopSwerve);
    swerve.resetPose();
    configureBindings();
  }

  private void configureBindings() {
    zeroTrigger.onTrue(new InstantCommand(() -> swerve.resetPose()));
    faceAngleTrigger.whileTrue(faceAngle);
    targetTrigger.whileTrue(intakeCommandGroup);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
