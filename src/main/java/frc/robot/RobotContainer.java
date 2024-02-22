// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AimShooterCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ExpelTrapNoteCommand;
import frc.robot.commands.HomeTrapArmCommand;
import frc.robot.commands.IntakeCommandGroup;
import frc.robot.commands.ManualTrapCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterHeightPIDSubsystem;
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
  private TrapHeightPIDSubsystem trapHeightPIDSubsystem = new TrapHeightPIDSubsystem();
  private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private ShooterHeightPIDSubsystem shooterHeightPIDSubsystem = new ShooterHeightPIDSubsystem();
  
  // commands
  private AimShooterCommand aimShooterCommand = new AimShooterCommand(swerve, driveController, shooterSubsystem, shooterHeightPIDSubsystem);
  private TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, driveController);
  private IntakeCommandGroup intakeCommandGroup = new IntakeCommandGroup(swerve, intakeSubsystem);
  private ExpelTrapNoteCommand expelTrapNoteCommand = new ExpelTrapNoteCommand(trapMechanismSubsystem);
  private ClimbCommand climbCommand = new ClimbCommand(climberSubsystem, secondaryController);
  private ManualTrapCommand manualTrapCommand = new ManualTrapCommand(secondaryController, trapHeightPIDSubsystem);
  private HomeTrapArmCommand homeTrapArmCommand = new HomeTrapArmCommand(trapHeightPIDSubsystem);
  
  // autos
  private PathPlannerAuto ampAuto;
  private PathPlannerAuto sourceAuto;

  // swerve/movement triggers
  private Trigger zeroPose = driveController.x();
  private Trigger ampTrigger = driveController.a();
  private Trigger sourceTrigger = driveController.b();
  // trap triggers
  private Trigger feedTrapTrigger = secondaryController.povRight();
  private Trigger expelTrapTrigger = secondaryController.povLeft();
  private Trigger toggleTrapTrigger = secondaryController.x();
  private Trigger homeTrapTrigger = secondaryController.start();
  // shooter triggers
  private Trigger aimShooterTrigger = driveController.leftBumper();
  private Trigger raiseShooterTrigger = secondaryController.y();
  private Trigger lowerShooterTrigger = secondaryController.a();
  private Trigger spinShooterTrigger = driveController.rightBumper();
  // intake triggers
  private Trigger intakeTrigger = secondaryController.rightTrigger();
  private Trigger reverseIntakeTrigger = secondaryController.leftBumper();
  private Trigger intakeFeedTrigger = secondaryController.rightBumper();
  private Trigger shooterFeedTrigger = driveController.rightTrigger().or(secondaryController.leftTrigger());

  public RobotContainer() {
    swerve.setDefaultCommand(teleopSwerve);
    climberSubsystem.setDefaultCommand(climbCommand);
    trapHeightPIDSubsystem.setDefaultCommand(manualTrapCommand);
    swerve.resetPose();
    configureAuto();
    configureBindings();
  }

  private void configureBindings() {
    // swerve/movement bindings
    zeroPose.onTrue(new InstantCommand(swerve::resetPose));
    ampTrigger.whileTrue(ampAuto);
    sourceTrigger.whileTrue(sourceAuto);
    // trap bindings
    feedTrapTrigger.whileTrue(new StartEndCommand(trapMechanismSubsystem::intake, trapMechanismSubsystem::stopRollers, trapMechanismSubsystem));
    expelTrapTrigger.whileTrue(new StartEndCommand(trapMechanismSubsystem::expel, trapMechanismSubsystem::stopRollers, trapMechanismSubsystem));
    toggleTrapTrigger.onTrue(new InstantCommand(trapMechanismSubsystem::toggleActuator));
    homeTrapTrigger.whileTrue(homeTrapArmCommand);
    // shooter bindings
    aimShooterTrigger.whileTrue(aimShooterCommand);
    raiseShooterTrigger.onTrue(new InstantCommand(shooterHeightPIDSubsystem::increaseAngle));
    lowerShooterTrigger.onTrue(new InstantCommand(shooterHeightPIDSubsystem::decreaseAngle));
    spinShooterTrigger.whileTrue(new StartEndCommand(shooterSubsystem::start, shooterSubsystem::stop, shooterSubsystem));
    // intake bindings
    intakeTrigger.whileTrue(intakeCommandGroup);
    reverseIntakeTrigger.whileTrue(new StartEndCommand(intakeSubsystem::reverse, intakeSubsystem::stopFeeding, intakeSubsystem));
    intakeFeedTrigger.whileTrue(new StartEndCommand(intakeSubsystem::startAll, intakeSubsystem::stopFeeding, intakeSubsystem));
    shooterFeedTrigger.whileTrue(new StartEndCommand(intakeSubsystem::startShooterFeed, intakeSubsystem::stopFeeding, intakeSubsystem));
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
    
    // bind commands
    NamedCommands.registerCommand("Raise To Amp", new InstantCommand(trapHeightPIDSubsystem::moveToAmp));
    NamedCommands.registerCommand("Raise To Source", new InstantCommand(trapHeightPIDSubsystem::moveToSource));
    NamedCommands.registerCommand("Flip Trap Down", new InstantCommand(trapMechanismSubsystem::extendActuator));
    NamedCommands.registerCommand("Flip Trap Up", new InstantCommand(trapMechanismSubsystem::contractActuator));
    NamedCommands.registerCommand("Expel Trap Note", expelTrapNoteCommand);
    NamedCommands.registerCommand("Intake Trap Command", new InstantCommand(trapMechanismSubsystem::intake));
    NamedCommands.registerCommand("Lower Trap Arm", new InstantCommand(trapHeightPIDSubsystem::moveToHome));
    NamedCommands.registerCommand("Move Actuator To Amp", new InstantCommand(trapMechanismSubsystem::moveActuatorForAmp));

    ampAuto = new PathPlannerAuto("Amp");
    sourceAuto = new PathPlannerAuto("Source");
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
