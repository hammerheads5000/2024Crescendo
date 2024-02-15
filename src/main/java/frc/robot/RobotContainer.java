// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TrapConstants;
import frc.robot.commands.ExpelTrapNoteCommand;
import frc.robot.commands.FaceSpeaker;
import frc.robot.commands.IntakeCommandGroup;
import frc.robot.commands.LowerArmCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TrapHeightPIDSubsystem;
import frc.robot.subsystems.TrapMechanismSubsystem;

public class RobotContainer {
  private CommandXboxController controller = new CommandXboxController(0);

  // subsystems
  private Swerve swerve = new Swerve();
  private AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem();
  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private TrapMechanismSubsystem trapMechanismSubsystem = new TrapMechanismSubsystem();
  private TrapHeightPIDSubsystem trapPIDSubsystem = new TrapHeightPIDSubsystem();
  
  // commands
  private FaceSpeaker faceAngle = new FaceSpeaker(swerve, controller);
  private TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, controller);
  private IntakeCommandGroup intakeCommandGroup = new IntakeCommandGroup(swerve, intakeSubsystem);
  private ExpelTrapNoteCommand expelTrapNoteCommand = new ExpelTrapNoteCommand(trapMechanismSubsystem);
  private LowerArmCommand lowerArmCommand = new LowerArmCommand(trapPIDSubsystem);
  
  // triggers
  private Trigger zeroTrigger = controller.y();

  private Trigger aimShooterTrigger = controller.leftBumper();
  private Trigger intakeTrigger = controller.rightBumper();

  private PathPlannerAuto ampAuto = new PathPlannerAuto("Amp");

  public RobotContainer() {
    swerve.setDefaultCommand(teleopSwerve);
    swerve.resetPose();
    configureBindings();
    configureAuto();
  }

  private void configureBindings() {
    zeroTrigger.onTrue(new InstantCommand(() -> swerve.resetPose()));
    aimShooterTrigger.whileTrue(aimShooterCommand);
    intakeTrigger.whileTrue(intakeCommandGroup);
  }

  private void configureAuto() {
    AutoBuilder.configureHolonomic(
      swerve::getPose,
      swerve::resetPose,
      swerve::getChassisSpeeds,
      swerve::driveRobotCentric,
      AutoConstants.holonomicPathFollowerConfig,
      () -> {
              // Boolean supplier to return true if on red alliance
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
      swerve);
      
      NamedCommands.registerCommand("Raise To Amp", new InstantCommand(() -> trapPIDSubsystem.setSetpoint(TrapConstants.ampPosition.in(Inches))));
      NamedCommands.registerCommand("Flip Trap Down", new InstantCommand(() -> trapMechanismSubsystem.extendActuator()));
      NamedCommands.registerCommand("Expel Trap Note", expelTrapNoteCommand);
      NamedCommands.registerCommand("Lower Trap Arm", lowerArmCommand);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
