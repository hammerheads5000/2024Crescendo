// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AimShooterCommand;
import frc.robot.commands.IntakeCommandGroup;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private Swerve swerve = new Swerve();
  private CommandXboxController controller = new CommandXboxController(0);
  private TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, controller);
  private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  
  private AimShooterCommand aimShooterCloseCommand = new AimShooterCommand(swerve, controller, shooterSubsystem, true);
  private AimShooterCommand aimShooterFarCommand = new AimShooterCommand(swerve, controller, shooterSubsystem, false);
 
  private AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem(); // DO NOT REMOVE. periodic is needed

  private Trigger zeroTrigger = controller.y();
  private Trigger aimShooterCloseTrigger = controller.leftBumper();
  private Trigger aimShooterFarTrigger = controller.leftTrigger();
  private Trigger intakeTrigger = controller.rightBumper();

  private IntakeCommandGroup intakeCommandGroup = new IntakeCommandGroup(swerve);

  public RobotContainer() {
    swerve.setDefaultCommand(teleopSwerve);
    swerve.resetPose();
    configureBindings();
  }

  private void configureBindings() {
    zeroTrigger.onTrue(new InstantCommand(() -> swerve.resetPose()));
    aimShooterCloseTrigger.whileTrue(aimShooterCloseCommand);
    aimShooterFarTrigger.whileTrue(aimShooterFarCommand);
    intakeTrigger.whileTrue(intakeCommandGroup);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
