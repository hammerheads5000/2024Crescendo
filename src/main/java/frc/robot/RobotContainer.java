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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TrapConstants;
import frc.robot.commands.AimShooterCommand;
import frc.robot.commands.ExpelTrapNoteCommand;
import frc.robot.commands.IntakeCommandGroup;
import frc.robot.commands.LowerArmCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TrapHeightPIDSubsystem;
import frc.robot.subsystems.TrapMechanismSubsystem;

public class RobotContainer {
  private CommandXboxController driveController = new CommandXboxController(0);
  private CommandXboxController secondaryController = new CommandXboxController(1);

  // subsystems
  private Swerve swerve = new Swerve();
  private AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem(); // DO NOT REMOVE. Need periodic
  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private TrapMechanismSubsystem trapMechanismSubsystem = new TrapMechanismSubsystem();
  private TrapHeightPIDSubsystem trapPIDSubsystem = new TrapHeightPIDSubsystem();
  private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  
  // commands
  private AimShooterCommand aimShooterCommand = new AimShooterCommand(swerve, driveController, shooterSubsystem);
  private TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, driveController);
  private IntakeCommandGroup intakeCommandGroup = new IntakeCommandGroup(swerve, intakeSubsystem);
  private ExpelTrapNoteCommand expelTrapNoteCommand = new ExpelTrapNoteCommand(trapMechanismSubsystem);
  private LowerArmCommand lowerArmCommand = new LowerArmCommand(trapPIDSubsystem);
  
  // autos
  private PathPlannerAuto ampAuto = new PathPlannerAuto("Amp");

  // triggers
  private Trigger ampTrigger = driveController.a();
  private Trigger aimShooterTrigger = driveController.leftBumper();
  private Trigger intakeTrigger = secondaryController.rightTrigger();
  private Trigger intakeFeedTrigger = secondaryController.rightBumper();
  private Trigger shooterFeedTrigger = secondaryController.leftTrigger();
  private Trigger raiseTrapTrigger = secondaryController.povUp();
  private Trigger lowerTrapTrigger = secondaryController.povDown();
  private Trigger feedTrapTrigger = secondaryController.povRight();
  private Trigger expelTrapTrigger = secondaryController.povLeft();
  private Trigger toggleTrapTrigger = secondaryController.x();
  private Trigger climbTrigger = secondaryController.y();
  private Trigger climbDownTrigger = secondaryController.b();

  public RobotContainer() {
    swerve.setDefaultCommand(teleopSwerve);
    swerve.resetPose();
    configureBindings();
    configureAuto();
  }

  private void configureBindings() {
    aimShooterTrigger.whileTrue(aimShooterCommand);
    intakeTrigger.whileTrue(intakeCommandGroup);
    intakeFeedTrigger.whileTrue(new StartEndCommand(intakeSubsystem::startAll, intakeSubsystem::stopFeeding, intakeSubsystem));
    ampTrigger.whileTrue(ampAuto);
    shooterFeedTrigger.whileTrue(new StartEndCommand(intakeSubsystem::startShooterFeed, intakeSubsystem::stopFeeding, intakeSubsystem));
    raiseTrapTrigger.whileTrue(new StartEndCommand(trapPIDSubsystem::raise, trapPIDSubsystem::stop, trapPIDSubsystem));
    lowerTrapTrigger.whileTrue(new StartEndCommand(trapPIDSubsystem::lower, trapPIDSubsystem::stop, trapPIDSubsystem));
    feedTrapTrigger.whileTrue(new StartEndCommand(trapMechanismSubsystem::intake, trapMechanismSubsystem::stopRollers, trapMechanismSubsystem));
    expelTrapTrigger.whileTrue(new StartEndCommand(trapMechanismSubsystem::expel, trapMechanismSubsystem::stopRollers, trapMechanismSubsystem));
    toggleTrapTrigger.onTrue(new InstantCommand(() -> {if (trapMechanismSubsystem.getActuator()==1) trapMechanismSubsystem.contractActuator(); else trapMechanismSubsystem.extendActuator();}));
    climbTrigger.onTrue(new RunCommand(climberSubsystem::climbUp, climberSubsystem));
    climbDownTrigger.onTrue(new RunCommand(climberSubsystem::climbDown, climberSubsystem));
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
