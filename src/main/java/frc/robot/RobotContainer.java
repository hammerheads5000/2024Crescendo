// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Color;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.autos.PickUpNoteAndShootCommand;
import frc.robot.commands.autos.ShootNoteCommand;
import frc.robot.commands.intake.IntakeCommandGroup;
import frc.robot.commands.shooter.AimShooterCommand;
import frc.robot.commands.shooter.SpinShooterCommand;
import frc.robot.commands.trapmechanism.AmpCommandGroup;
import frc.robot.commands.trapmechanism.AutoTrapCommand;
import frc.robot.commands.trapmechanism.ExpelTrapNoteCommand;
import frc.robot.commands.trapmechanism.HomeTrapArmCommand;
import frc.robot.commands.trapmechanism.IntakeTrapNoteCommandGroup;
import frc.robot.commands.trapmechanism.ManualTrapCommand;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.ShooterHeightPIDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.trapmechanism.TrapHeightPIDSubsystem;
import frc.robot.subsystems.trapmechanism.TrapMechanismSubsystem;

public class RobotContainer {

  CommandXboxController driveController = new CommandXboxController(0);
  CommandXboxController secondaryController = new CommandXboxController(1);

  // subsystems
  Swerve swerve = new Swerve();
  AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem(); // DO NOT REMOVE. Need periodic
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem(); 

  TrapMechanismSubsystem trapMechanismSubsystem = new TrapMechanismSubsystem();
  TrapHeightPIDSubsystem trapHeightPIDSubsystem = new TrapHeightPIDSubsystem();

  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  ShooterHeightPIDSubsystem shooterHeightPIDSubsystem = new ShooterHeightPIDSubsystem();

  ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  
  private LightsSubsystem lightsSubsystem = new LightsSubsystem();
  // commands

  TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, driveController);
  Command intakeCommandGroup = new IntakeCommandGroup(swerve, intakeSubsystem).handleInterrupt(intakeSubsystem::stopAll);
  Command expelTrapNoteCommand = new ExpelTrapNoteCommand(trapMechanismSubsystem)
      .handleInterrupt(trapMechanismSubsystem::stopRollers); // stop on interrupt  
  HomeTrapArmCommand homeTrapArmCommand = new HomeTrapArmCommand(trapHeightPIDSubsystem);
  ManualTrapCommand manualTrapCommand = new ManualTrapCommand(secondaryController, trapHeightPIDSubsystem);
  Command intakeTrapNoteCommand = new IntakeTrapNoteCommandGroup(trapMechanismSubsystem, trapHeightPIDSubsystem)
      .handleInterrupt(trapMechanismSubsystem::stopRollers); // stop on interrupt
  
  AimShooterCommand aimShooterCommand = new AimShooterCommand(swerve, driveController, shooterHeightPIDSubsystem);
  SpinShooterCommand spinShooterCommand = new SpinShooterCommand(shooterSubsystem);
 

  ClimbCommand climbCommand = new ClimbCommand(climberSubsystem, secondaryController, shooterHeightPIDSubsystem);
  AutoTrapCommand autoTrapCommand = new AutoTrapCommand(trapHeightPIDSubsystem, trapMechanismSubsystem, climberSubsystem);
  Command ampCommandGroup = new AmpCommandGroup(trapMechanismSubsystem, trapHeightPIDSubsystem)
      .handleInterrupt(trapMechanismSubsystem::stopRollers); // stop on interrupt
  // autos
  Command ampAuto;
  Command sourceAuto;
  SendableChooser<Command> autoChooser;
  Command autoStartCommand = new ShootNoteCommand(swerve, intakeSubsystem, shooterSubsystem, shooterHeightPIDSubsystem);

  // swerve/movement triggers
  Trigger zeroPose = driveController.x();
  Trigger ampTrigger = driveController.a();
  Trigger sourceTrigger = driveController.b();
  // trap triggers
  Trigger feedTrapTrigger = secondaryController.povRight();
  Trigger expelTrapTrigger = secondaryController.povLeft();
  Trigger toggleTrapTrigger = secondaryController.x();
  Trigger homeTrapTrigger = secondaryController.start();
  Trigger moveAmpTrigger = secondaryController.axisGreaterThan(5, Constants.controllerDeadband)
                                .or(secondaryController.axisLessThan(5, -Constants.controllerDeadband)); // right joystick moved
  Trigger AutoSourceTrigger = secondaryController.povUp();
  Trigger AutoTrapTrigger = secondaryController.povDown();
  Trigger Amptrigger = secondaryController.back();
  // shooter triggers
  Trigger aimShooterTrigger = driveController.leftBumper();
  Trigger raiseShooterTrigger = secondaryController.y();
  Trigger lowerShooterTrigger = secondaryController.a();
  Trigger spinShooterTrigger = driveController.rightBumper();
  Trigger closeShootTrigger = driveController.leftTrigger();
  // intake triggers
  Trigger intakeTrigger = secondaryController.rightTrigger();
  Trigger reverseIntakeTrigger = secondaryController.leftBumper();
  Trigger intakeFeedTrigger = secondaryController.rightBumper();
  Trigger shooterFeedTrigger = driveController.rightTrigger().or(secondaryController.leftTrigger());
  // climb triggers
  Trigger climbTrigger = secondaryController.b().and(
      secondaryController.axisGreaterThan(1, Constants.controllerDeadband)
          .or(secondaryController.axisLessThan(1, -Constants.controllerDeadband))); // left joystick y moved while b held

          private AmpCommandGroup ampCommandGroup = new AmpCommandGroup(trapMechanismSubsystem, trapHeightPIDSubsystem, expelTrapTrigger);
        
  // Light Triggers
  private Trigger blueLightTrigger = buttonBoardOne.button(1);
  private Trigger redLightTrigger = buttonBoardOne.button(2);
  private Trigger testLightButton = buttonBoardOne.button(3);



  public RobotContainer() {
    swerve.setDefaultCommand(teleopSwerve);
    swerve.resetPose();
    configureAuto();
    configureBindings();
  }

  void configureBindings() {
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
    AutoSourceTrigger.whileTrue(intakeTrapNoteCommand);
    AutoTrapTrigger.whileTrue(autoTrapCommand);
    Amptrigger.whileTrue(ampCommandGroup);
    // shooter bindings
    aimShooterTrigger.whileTrue(aimShooterCommand);
    raiseShooterTrigger.onTrue(new InstantCommand(shooterHeightPIDSubsystem::increaseAngle));
    lowerShooterTrigger.onTrue(new InstantCommand(shooterHeightPIDSubsystem::decreaseAngle));
    spinShooterTrigger.whileTrue(spinShooterCommand);
    closeShootTrigger.onTrue(new InstantCommand(() -> shooterHeightPIDSubsystem.setTargetAngle(ShooterConstants.closeAngle)));
    // intake bindings
    intakeTrigger.whileTrue(intakeCommandGroup);
    reverseIntakeTrigger.whileTrue(new StartEndCommand(intakeSubsystem::reverse, intakeSubsystem::stopAll, intakeSubsystem));
    intakeFeedTrigger.whileTrue(new StartEndCommand(intakeSubsystem::startAll, intakeSubsystem::stopAll, intakeSubsystem));
    shooterFeedTrigger.whileTrue(new StartEndCommand(intakeSubsystem::startShooterFeed, intakeSubsystem::stopAll, intakeSubsystem));
    // climb bindings
    climbTrigger.whileTrue(climbCommand);

    // light bindings
    blueLightTrigger.onTrue(new InstantCommand(() -> lightsSubsystem.SetSolidColor(Color.kBlue)));
    redLightTrigger.onTrue(new InstantCommand(() -> lightsSubsystem.setSectionColor(0, 8/3, Color.kCrimson)));
    testLightButton.onTrue(new InstantCommand(lightsSubsystem::TestColor));
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
    NamedCommands.registerCommand("Raise To Amp", new InstantCommand(trapHeightPIDSubsystem::moveToAmp));
    NamedCommands.registerCommand("Raise To Source", new InstantCommand(trapHeightPIDSubsystem::moveToSource));
    NamedCommands.registerCommand("Flip Trap Down", new InstantCommand(trapMechanismSubsystem::extendActuator));
    NamedCommands.registerCommand("Flip Trap Up", new InstantCommand(trapMechanismSubsystem::contractActuator));
    NamedCommands.registerCommand("Expel Trap Note", expelTrapNoteCommand);
    NamedCommands.registerCommand("Intake Trap Note", intakeTrapNoteCommand);
    NamedCommands.registerCommand("Lower Trap Arm", new InstantCommand(trapHeightPIDSubsystem::moveToHome));
    NamedCommands.registerCommand("Move Actuator To Amp", new InstantCommand(trapMechanismSubsystem::moveActuatorForAmp));

    NamedCommands.registerCommand("Pick Up Note and Shoot", new PickUpNoteAndShootCommand(swerve, intakeSubsystem, shooterSubsystem, shooterHeightPIDSubsystem));
    NamedCommands.registerCommand("Pick Up Note", intakeCommandGroup);
    NamedCommands.registerCommand("Shoot", new ShootNoteCommand(swerve, intakeSubsystem, shooterSubsystem, shooterHeightPIDSubsystem));

    ampAuto = AutoBuilder.buildAuto("Amp");
    sourceAuto = AutoBuilder.buildAuto("Source");

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
  }

  public Command getAutonomousCommand() {
    return spinShooterCommand.alongWith(autoStartCommand.andThen(autoChooser.getSelected()));
  }
}
