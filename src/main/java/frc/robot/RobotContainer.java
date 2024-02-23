// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.intake.IntakeCommandGroup;
import frc.robot.commands.trapmechanism.ExpelTrapNoteCommand;
import frc.robot.commands.trapmechanism.HomeTrapArmCommand;
import frc.robot.commands.trapmechanism.IntakeTrapNoteCommandGroup;
import frc.robot.commands.trapmechanism.ManualTrapCommand;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.ShooterHeightPIDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.trapmechanism.TrapHeightPIDSubsystem;
import frc.robot.subsystems.trapmechanism.TrapMechanismSubsystem;

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
  private Command intakeTrapNoteCommand = new IntakeTrapNoteCommandGroup(trapMechanismSubsystem)
      .handleInterrupt(trapMechanismSubsystem::stopRollers); // stop on interrupt
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
  private Trigger moveAmpTrigger = secondaryController.axisGreaterThan(5, Constants.controllerDeadband)
                                .or(secondaryController.axisLessThan(5, -Constants.controllerDeadband)); // right joystick moved
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
    swerve.resetPose();
    configureAuto();
    configureBindings();
  }

  private void configureBindings() {
    // swerve/movement bindings
    zeroPose.onTrue(new InstantCommand(swerve::resetPose));
    ampTrigger.whileTrue(ampAuto);
    ampTrigger.onTrue(new InstantCommand(trapHeightPIDSubsystem::moveToAmp));
    sourceTrigger.whileTrue(sourceAuto);
    // trap bindings
    feedTrapTrigger.whileTrue(new StartEndCommand(trapMechanismSubsystem::forward, trapMechanismSubsystem::stopRollers, trapMechanismSubsystem));
    expelTrapTrigger.whileTrue(new StartEndCommand(trapMechanismSubsystem::reverse, trapMechanismSubsystem::stopRollers, trapMechanismSubsystem));
    toggleTrapTrigger.onTrue(new InstantCommand(trapMechanismSubsystem::toggleActuator));
    homeTrapTrigger.whileTrue(homeTrapArmCommand);
    moveAmpTrigger.whileTrue(manualTrapCommand);
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
    NamedCommands.registerCommand("Intake Trap Note", intakeTrapNoteCommand);
    NamedCommands.registerCommand("Lower Trap Arm", new InstantCommand(trapHeightPIDSubsystem::moveToHome));
    NamedCommands.registerCommand("Move Actuator To Amp", new InstantCommand(trapMechanismSubsystem::moveActuatorForAmp));

    ampAuto = new PathPlannerAuto("Amp");
    sourceAuto = new PathPlannerAuto("Source");
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
