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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TrapConstants;
import frc.robot.commands.AimShooterCommand;
import frc.robot.commands.ExpelTrapNoteCommand;
import frc.robot.commands.IntakeCommandGroup;
import frc.robot.commands.LowerTrapArmCommand;
import frc.robot.commands.RaiseTrapArmCommand;
import frc.robot.commands.RaiseTrapArmToPositionCommand;
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
  private LowerTrapArmCommand lowerArmCommand = new LowerTrapArmCommand(trapHeightPIDSubsystem);
  private RaiseTrapArmCommand raiseArmCommand = new RaiseTrapArmCommand(trapHeightPIDSubsystem);
  
  // autos
  private PathPlannerAuto ampAuto;
  private PathPlannerAuto sourceAuto;

  // triggers
  private Trigger zeroPose = driveController.x();
  private Trigger ampTrigger = driveController.a();
  private Trigger sourceTrigger = driveController.b();
  private Trigger aimShooterTrigger = driveController.leftBumper();
  // trap triggers
  private Trigger raiseTrapTrigger = secondaryController.povUp();
  private Trigger lowerTrapTrigger = secondaryController.povDown();
  private Trigger feedTrapTrigger = secondaryController.povRight();
  private Trigger expelTrapTrigger = secondaryController.povLeft();
  private Trigger toggleTrapTrigger = secondaryController.x();
  // climb triggers
  private Trigger climbTrigger = driveController.y();
  private Trigger climbDownTrigger = driveController.b();
  // manual shooter triggers
  private Trigger raiseShooterTrigger = secondaryController.y();
  private Trigger lowerShooterTrigger = secondaryController.a();
  private Trigger spinShooterTrigger = driveController.rightBumper();
  private Trigger reverseIntakeTrigger = secondaryController.leftBumper();
  private Trigger intakeTrigger = secondaryController.rightTrigger();
  private Trigger intakeFeedTrigger = secondaryController.rightBumper();
  private Trigger shooterFeedTrigger = driveController.rightTrigger().or(secondaryController.leftTrigger());

  public RobotContainer() {
    swerve.setDefaultCommand(teleopSwerve);
    swerve.resetPose();
    configureAuto();
    configureBindings();
  }

  private void configureBindings() {
    zeroPose.onTrue(new InstantCommand(() -> swerve.resetPose()));
    aimShooterTrigger.whileTrue(aimShooterCommand);
    intakeTrigger.whileTrue(intakeCommandGroup);
    intakeFeedTrigger.whileTrue(new StartEndCommand(intakeSubsystem::startAll, intakeSubsystem::stopFeeding, intakeSubsystem));
    ampTrigger.whileTrue(ampAuto);
    sourceTrigger.whileTrue(sourceAuto);
    shooterFeedTrigger.whileTrue(new StartEndCommand(intakeSubsystem::startShooterFeed, intakeSubsystem::stopFeeding, intakeSubsystem));
    raiseTrapTrigger.whileTrue(raiseArmCommand);
    lowerTrapTrigger.whileTrue(lowerArmCommand);
    feedTrapTrigger.whileTrue(new StartEndCommand(trapMechanismSubsystem::intake, trapMechanismSubsystem::stopRollers, trapMechanismSubsystem));
    expelTrapTrigger.whileTrue(new StartEndCommand(trapMechanismSubsystem::expel, trapMechanismSubsystem::stopRollers, trapMechanismSubsystem));
    toggleTrapTrigger.onTrue(new InstantCommand(trapMechanismSubsystem::toggleActuator));
    climbTrigger.onTrue(new RunCommand(climberSubsystem::climbUp, climberSubsystem));
    climbDownTrigger.onTrue(new RunCommand(climberSubsystem::climbDown, climberSubsystem));
    spinShooterTrigger.whileTrue(new StartEndCommand(shooterSubsystem::start, shooterSubsystem::stop, shooterSubsystem));
    raiseShooterTrigger.onTrue(new InstantCommand(shooterHeightPIDSubsystem::increaseAngle));
    lowerShooterTrigger.onTrue(new InstantCommand(shooterHeightPIDSubsystem::decreaseAngle));
    reverseIntakeTrigger.whileTrue(new StartEndCommand(intakeSubsystem::reverse, intakeSubsystem::stopFeeding, intakeSubsystem));
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
      
      NamedCommands.registerCommand("Raise To Amp", new RaiseTrapArmToPositionCommand(trapHeightPIDSubsystem, TrapConstants.ampPosition));
      NamedCommands.registerCommand("Raise To Source", new InstantCommand(trapHeightPIDSubsystem::moveToSource));
      NamedCommands.registerCommand("Raise To Amp", new InstantCommand(trapHeightPIDSubsystem::moveToAmp));
      NamedCommands.registerCommand("Flip Trap Down", new InstantCommand(trapMechanismSubsystem::extendActuator));
      NamedCommands.registerCommand("Flip Trap Up", new InstantCommand(trapMechanismSubsystem::contractActuator));
      NamedCommands.registerCommand("Expel Trap Note", expelTrapNoteCommand);
      NamedCommands.registerCommand("Intake Trap Command", new InstantCommand(trapMechanismSubsystem::intake));
      NamedCommands.registerCommand("Lower Trap Arm", lowerArmCommand);

    ampAuto = new PathPlannerAuto("Amp");
    sourceAuto = new PathPlannerAuto("Source");
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
