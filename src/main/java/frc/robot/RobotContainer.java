// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.autos.PickUpNoteAndShootCommand;
import frc.robot.commands.autos.ShootNoteCommand;
import frc.robot.commands.intake.FinishIntakeCommand;
import frc.robot.commands.intake.IntakeCommandGroup;
import frc.robot.commands.intake.ManualIntakeCommand;
import frc.robot.commands.lights.DisabledLightsCommand;
import frc.robot.commands.shooter.AimShooterCommand;
import frc.robot.commands.shooter.SpinShooterCommand;
import frc.robot.commands.trapmechanism.AmpCommandGroup;
import frc.robot.commands.trapmechanism.AutoTrapCommand;
import frc.robot.commands.trapmechanism.AutoTrapHomeCommandGroup;
import frc.robot.commands.trapmechanism.ExpelTrapNoteCommand;
import frc.robot.commands.trapmechanism.HomeTrapArmCommand;
import frc.robot.commands.trapmechanism.IntakeTrapNoteCommandGroup;
import frc.robot.commands.trapmechanism.ManualRollerCommand;
import frc.robot.commands.trapmechanism.ManualTrapCommand;
import frc.robot.commands.trapmechanism.ManyBonksCommand;
import frc.robot.commands.trapmechanism.ReverseTrapCommand;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.BonkerSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.PowerSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.ShooterHeightPIDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.trapmechanism.TrapHeightPIDSubsystem;
import frc.robot.subsystems.trapmechanism.TrapMechanismSubsystem;

public class RobotContainer {

  CommandXboxController driveController = new CommandXboxController(0);
  CommandXboxController secondaryController = new CommandXboxController(1);
  CommandJoystick buttonBoardOne = new CommandJoystick(2);
  CommandJoystick buttonBoardTwo = new CommandJoystick(3);
  // subsystems
  LightsSubsystem lightsSubsystem = new LightsSubsystem();
  Swerve swerve = new Swerve();
  AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem(); // DO NOT REMOVE. Need periodic
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem(); 
  BonkerSubsystem bonkerSubsystem = new BonkerSubsystem();
  TrapMechanismSubsystem trapMechanismSubsystem = new TrapMechanismSubsystem();
  TrapHeightPIDSubsystem trapHeightPIDSubsystem = new TrapHeightPIDSubsystem();

  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  ShooterHeightPIDSubsystem shooterHeightPIDSubsystem = new ShooterHeightPIDSubsystem();

  ClimberSubsystem climberSubsystem = new ClimberSubsystem(lightsSubsystem);
  PowerSubsystem powerSubsystem = new PowerSubsystem();

  // commands
  ReverseTrapCommand reverseTrapCommand = new ReverseTrapCommand(trapMechanismSubsystem);
  TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, driveController);
  Command intakeCommandGroup = new IntakeCommandGroup(swerve, intakeSubsystem, lightsSubsystem).handleInterrupt(intakeSubsystem::stopAll);
  Command manualIntakeCommand = new ManualIntakeCommand(intakeSubsystem, lightsSubsystem).handleInterrupt(intakeSubsystem::stopAll);
  Command finishIntakeCommmand = new FinishIntakeCommand(intakeSubsystem, lightsSubsystem).handleInterrupt(intakeSubsystem::stopAll);
  Command expelTrapNoteCommand = new ExpelTrapNoteCommand(trapMechanismSubsystem)
      .handleInterrupt(trapMechanismSubsystem::stopRollers); // stop on interrupt  
  HomeTrapArmCommand homeTrapArmCommand = new HomeTrapArmCommand(trapHeightPIDSubsystem);
  ManualTrapCommand manualTrapCommand = new ManualTrapCommand(secondaryController, trapHeightPIDSubsystem);
  Command intakeTrapNoteCommand = new IntakeTrapNoteCommandGroup(trapMechanismSubsystem, trapHeightPIDSubsystem, lightsSubsystem)
      .handleInterrupt(trapMechanismSubsystem::stopRollers); // stop on interrupt
  DisabledLightsCommand disabledLightsCommand = new DisabledLightsCommand(lightsSubsystem, aprilTagSubsystem, powerSubsystem);
  AimShooterCommand aimShooterCommand = new AimShooterCommand(swerve, driveController, shooterHeightPIDSubsystem, lightsSubsystem);
  SpinShooterCommand spinShooterCommand = new SpinShooterCommand(shooterSubsystem, lightsSubsystem);
  AutoTrapHomeCommandGroup autoTrapHomeCommandGroup = new AutoTrapHomeCommandGroup(trapHeightPIDSubsystem, trapMechanismSubsystem);
  ClimbCommand climbCommand = new ClimbCommand(climberSubsystem, secondaryController, shooterHeightPIDSubsystem, trapHeightPIDSubsystem);
  Command autoTrapCommand = new AutoTrapCommand(trapHeightPIDSubsystem, trapMechanismSubsystem, climberSubsystem, shooterHeightPIDSubsystem, lightsSubsystem)
  .handleInterrupt(() -> {climberSubsystem.stopMotor(); trapMechanismSubsystem.stopRollers();});

  Command ampCommandGroup = new AmpCommandGroup(trapMechanismSubsystem, trapHeightPIDSubsystem)
      .handleInterrupt(trapMechanismSubsystem::stopRollers); // stop on interrupt
  ManualRollerCommand manualRollerCommand = new ManualRollerCommand(trapMechanismSubsystem, secondaryController);
  Command manyBonksCommand = new ManyBonksCommand(bonkerSubsystem).repeatedly();

  // autos
  Command ampAuto;
  Command sourceAuto;
  Command stage1Auto;
  Command stage2Auto;
  HashMap<Integer, Command> stageAutos = new HashMap<Integer, Command>();
  SendableChooser<Command> autoChooser;
  Command autoStartCommand = new ShootNoteCommand(swerve, intakeSubsystem, shooterSubsystem, shooterHeightPIDSubsystem, lightsSubsystem);

  // swerve/movement triggers
  Trigger zeroPose = driveController.x();
  Trigger ampTrigger = driveController.povRight();
  Trigger sourceTrigger = driveController.povLeft();
  Trigger fastTrigger = driveController.y();
  Trigger slowTrigger = driveController.a();
  // trap triggers
  Trigger feedTrapTrigger = buttonBoardOne.button(12).or(secondaryController.back());
  Trigger expelTrapTrigger = buttonBoardOne.button(7).or(secondaryController.start());
  Trigger toggleTrapTrigger = buttonBoardOne.button(9).or(secondaryController.x());
  Trigger homeTrapTrigger = buttonBoardOne.button(11);
  Trigger moveUpManualTrigger = buttonBoardTwo.button(9);
  Trigger moveDownManualTrigger = buttonBoardTwo.button(8);
  Trigger reverseTrapAutoTrigger = buttonBoardOne.button(1);
  Trigger AutoSourceTrigger = buttonBoardOne.button(8).or(secondaryController.povRight()).or(buttonBoardTwo.button(8));
  Trigger AutoTrapTrigger = buttonBoardOne.button(6).or(secondaryController.povLeft()).or(buttonBoardTwo.button(9));
  Trigger Amptrigger = buttonBoardOne.button(5).or(secondaryController.povUp());
  Trigger autoTrapToHomeTrigger = buttonBoardOne.button(10); 
  Trigger TrapMoveJoystickTrigger = secondaryController.axisLessThan(1, -Constants.controllerDeadband).or(secondaryController.axisGreaterThan(1, Constants.controllerDeadband));
  Trigger TrapRollerJoystickTrigger = secondaryController.axisGreaterThan(5, Constants.controllerDeadband).or(secondaryController.axisLessThan(5, -Constants.controllerDeadband));
  Trigger BonkerForwardButton = buttonBoardOne.button(2);
  Trigger BonkerBackWardButton = buttonBoardOne.button(4);
  Trigger manyBonksTrigger = buttonBoardTwo.button(12).or(secondaryController.b());
  Trigger TrapMechToTrapHeightTirgger = buttonBoardOne.button(3);
  // shooter triggers
  Trigger aimShooterTrigger = driveController.leftBumper();
  Trigger raiseShooterTrigger = secondaryController.y();
  Trigger lowerShooterTrigger = secondaryController.a();
  Trigger spinShooterTrigger = driveController.rightBumper();
  Trigger shooterLowAngleTrigger = driveController.povDown();
  Trigger shooterHighAngleTrigger = driveController.povUp();
  //Trigger spinShooterManualTrigger = driveController.b();
  // intake triggers
  Trigger intakeTrigger = secondaryController.rightTrigger();
  Trigger reverseIntakeTrigger = secondaryController.leftBumper();
  Trigger intakeFeedTrigger = secondaryController.rightBumper();
  Trigger shooterFeedTrigger = driveController.rightTrigger().or(secondaryController.leftTrigger());
  Trigger slowRollIntakeTrigger = buttonBoardTwo.button(4);
  // climb triggers
  Trigger climbUpTrigger = buttonBoardTwo.button(10);
  Trigger climbDownTrigger = buttonBoardTwo.button(11);
  Trigger climbAlignmentTrigger = driveController.b();
 
  public RobotContainer() {
    swerve.setDefaultCommand(teleopSwerve);
    swerve.resetPose();
    enablePhotonvisionPortForwarding();
    configureAuto();
    configureBindings();
  }

  void configureBindings() {
    // swerve/movement bindings
    zeroPose.onTrue(new InstantCommand(swerve::resetPose));
    ampTrigger.whileTrue(ampAuto);
    ampTrigger.onTrue(new InstantCommand(trapHeightPIDSubsystem::moveToAmp));
    sourceTrigger.whileTrue(sourceAuto);
    fastTrigger.whileTrue(new StartEndCommand(teleopSwerve::setFastSpeed, teleopSwerve::setDefaultSpeed, new Subsystem[0]));
    slowTrigger.whileTrue(new StartEndCommand(teleopSwerve::setSlowSpeed, teleopSwerve::setDefaultSpeed, new Subsystem[0]));
    // trap bindings
    feedTrapTrigger.whileTrue(new StartEndCommand(() -> trapMechanismSubsystem.moveManual(Constants.TrapConstants.slowExpelSpeed), trapMechanismSubsystem::stopRollers, trapMechanismSubsystem));
    expelTrapTrigger.whileTrue(new StartEndCommand(() -> trapMechanismSubsystem.moveManual(-Constants.TrapConstants.expelSpeed), trapMechanismSubsystem::stopRollers, trapMechanismSubsystem));
    toggleTrapTrigger.onTrue(new InstantCommand(trapMechanismSubsystem::toggleActuator));
    homeTrapTrigger.whileTrue(homeTrapArmCommand);
    AutoSourceTrigger.whileTrue(intakeTrapNoteCommand);
    AutoTrapTrigger.whileTrue(autoTrapCommand);
    Amptrigger.whileTrue(ampCommandGroup);
    autoTrapToHomeTrigger.whileTrue(autoTrapHomeCommandGroup);
    TrapMoveJoystickTrigger.whileTrue(manualTrapCommand);
    TrapRollerJoystickTrigger.whileTrue(climbCommand) ;
    //UnlockTrapSafetyTrigger.onTrue(new InstantCommand(trapHeightPIDSubsystem::disableSafety));
    //UnlockTrapSafetyTrigger.onTrue(new InstantCommand(trapMechanismSubsystem::disableSafety));
    //lockTrapSafetyTrigger.onTrue(new InstantCommand(trapHeightPIDSubsystem::enableSafety));
    reverseTrapAutoTrigger.whileTrue(reverseTrapCommand);
    BonkerBackWardButton.onTrue(new InstantCommand(bonkerSubsystem::bonkBackward));
    BonkerForwardButton.onTrue(new InstantCommand(bonkerSubsystem::bonkForward));
    manyBonksTrigger.whileTrue(manyBonksCommand);
    TrapMechToTrapHeightTirgger.whileTrue(new StartEndCommand(trapHeightPIDSubsystem::moveToTrap, trapHeightPIDSubsystem::stop, trapHeightPIDSubsystem));

    // shooter bindings
    aimShooterTrigger.whileTrue(aimShooterCommand);
    raiseShooterTrigger.onTrue(new InstantCommand(shooterHeightPIDSubsystem::increaseAngle));
    lowerShooterTrigger.onTrue(new InstantCommand(shooterHeightPIDSubsystem::decreaseAngle));
    spinShooterTrigger.whileTrue(spinShooterCommand);
    shooterLowAngleTrigger.onTrue(new InstantCommand(() -> shooterHeightPIDSubsystem.setTargetAngle(ShooterConstants.farAngle)));
    shooterHighAngleTrigger.onTrue(new InstantCommand(() -> shooterHeightPIDSubsystem.setTargetAngle(ShooterConstants.closeAngle)));
    //spinShooterManualTrigger.whileTrue(new InstantCommand(() -> shooterSubsystem.spin(Constants.ShooterConstants.manualShooterSpeed)));
    // intake bindings
    intakeTrigger.whileTrue(intakeCommandGroup);
    reverseIntakeTrigger.whileTrue(new StartEndCommand(intakeSubsystem::reverse, intakeSubsystem::stopAll, intakeSubsystem));
    intakeFeedTrigger.whileTrue(manualIntakeCommand);
    shooterFeedTrigger.whileTrue(new StartEndCommand(intakeSubsystem::startShooterFeed, intakeSubsystem::stopAll, intakeSubsystem));
    slowRollIntakeTrigger.whileTrue(new StartEndCommand(() -> intakeSubsystem.startAll(Constants.IntakeConstants.slowFeedRate), intakeSubsystem::stopAll, intakeSubsystem));
    
    // climb bindings
    climbUpTrigger.whileTrue(new StartEndCommand(() -> climberSubsystem.climb(Constants.ClimberConstants.climbSpeed), climberSubsystem::stopMotor, climberSubsystem));
    climbDownTrigger.whileTrue(new StartEndCommand(() -> climberSubsystem.climb(-Constants.ClimberConstants.climbSpeed), climberSubsystem::stopMotor, climberSubsystem));
    climbAlignmentTrigger.whileTrue(new ProxyCommand(() -> stageAutos.get(climberSubsystem.getStageAutoNumber())));
  }

  void configureAuto() {
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
    NamedCommands.registerCommand("Flip Trap Down", new InstantCommand(trapMechanismSubsystem::toggleActuator));
    NamedCommands.registerCommand("Raise To Amp", new InstantCommand(trapHeightPIDSubsystem::moveToAmp));
    NamedCommands.registerCommand("Raise To Source", new InstantCommand(trapHeightPIDSubsystem::moveToSource));
    NamedCommands.registerCommand("Flip Trap Down", new InstantCommand(trapMechanismSubsystem::extendActuator));
    NamedCommands.registerCommand("Flip Trap Up", new InstantCommand(trapMechanismSubsystem::contractActuator));
    NamedCommands.registerCommand("Expel Trap Note", CommandUtil.wrappedEventCommand(expelTrapNoteCommand));
    NamedCommands.registerCommand("Intake Trap Note", CommandUtil.wrappedEventCommand(intakeTrapNoteCommand));
    NamedCommands.registerCommand("Lower Trap Arm", new InstantCommand(trapHeightPIDSubsystem::moveToHome));
    NamedCommands.registerCommand("Move Actuator To Amp", new InstantCommand(trapMechanismSubsystem::moveActuatorForAmp));
    NamedCommands.registerCommand("Trap Climb", CommandUtil.wrappedEventCommand(autoTrapCommand));
    NamedCommands.registerCommand("Align Trap Note", CommandUtil.wrappedEventCommand(reverseTrapCommand));

    NamedCommands.registerCommand("Pick Up Note and Shoot", new PickUpNoteAndShootCommand(swerve, intakeSubsystem, shooterSubsystem, shooterHeightPIDSubsystem,lightsSubsystem));
    NamedCommands.registerCommand("Pick Up Note", CommandUtil.wrappedEventCommand(intakeCommandGroup));
    NamedCommands.registerCommand("Shoot", new ShootNoteCommand(swerve, intakeSubsystem, shooterSubsystem, shooterHeightPIDSubsystem, lightsSubsystem));
    NamedCommands.registerCommand("Start Intake", CommandUtil.wrappedEventCommand(manualIntakeCommand));
    NamedCommands.registerCommand("Finish Intake", new FinishIntakeCommand(intakeSubsystem, lightsSubsystem).onlyIf(() -> LoggingConstants.hasNoteSubscriber.get()));

    ampAuto = AutoBuilder.buildAuto("Amp");
    sourceAuto = AutoBuilder.buildAuto("Source");

    stage1Auto = AutoBuilder.buildAuto("Stage 1");
    stage2Auto = AutoBuilder.buildAuto("Stage 2");
    stageAutos.put(1, stage1Auto);
    stageAutos.put(2, stage2Auto);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
  }

  public Command getAutonomousCommand() {
    Command autoStartCommand = new ShootNoteCommand(swerve, intakeSubsystem, shooterSubsystem, shooterHeightPIDSubsystem, lightsSubsystem);
    Command spinShooterCommand = new SpinShooterCommand(shooterSubsystem, lightsSubsystem);
    return spinShooterCommand.alongWith(autoStartCommand.andThen(autoChooser.getSelected()));
  }

  public void enablePhotonvisionPortForwarding() {
    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, "10.50.0.11", port);
      PortForwarder.add(port, "10.50.0.205", port);
      PortForwarder.add(port, "photonvision.local", port);
    }
  }
}
