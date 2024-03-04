// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.SafeClimbCommandGroup;
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
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.ShooterHeightPIDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.trapmechanism.TrapHeightPIDSubsystem;
import frc.robot.subsystems.trapmechanism.TrapMechanismSubsystem;
import frc.robot.commands.trapmechanism.IntakeTrapNoteCommandGroup;

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
  private ShooterHeightPIDSubsystem shooterHeightPIDSubsystem = new ShooterHeightPIDSubsystem();

  private ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  
  // commands
  private TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, driveController);
  private Command intakeCommandGroup = new IntakeCommandGroup(swerve, intakeSubsystem).handleInterrupt(intakeSubsystem::stopAll);
  private Command safeClimbCommand = new SafeClimbCommandGroup(trapHeightPIDSubsystem, climberSubsystem, trapMechanismSubsystem);
  private ExpelTrapNoteCommand expelTrapNoteCommand = new ExpelTrapNoteCommand(trapMechanismSubsystem);
  private HomeTrapArmCommand homeTrapArmCommand = new HomeTrapArmCommand(trapHeightPIDSubsystem);
  private ManualTrapCommand manualTrapCommand = new ManualTrapCommand(secondaryController, trapHeightPIDSubsystem);
  private Command intakeTrapNoteCommand = new IntakeTrapNoteCommandGroup(trapMechanismSubsystem, trapHeightPIDSubsystem)
      .handleInterrupt(trapMechanismSubsystem::stopRollers); // stop on interrupt
  
  private AimShooterCommand aimShooterCommand = new AimShooterCommand(swerve, driveController, shooterHeightPIDSubsystem);
  private SpinShooterCommand spinShooterCommand = new SpinShooterCommand(shooterSubsystem);
 
  private ClimbCommand climbCommand = new ClimbCommand(climberSubsystem, secondaryController, shooterHeightPIDSubsystem);
  private AutoTrapCommand autoTrapCommand = new AutoTrapCommand(trapHeightPIDSubsystem, trapMechanismSubsystem, climberSubsystem);
  // autos
  private Command ampAuto;
  private Command sourceAuto;
  private SendableChooser<Command> autoChooser;
  private Command autoStartCommand = new ShootNoteCommand(swerve, intakeSubsystem, shooterSubsystem, shooterHeightPIDSubsystem);

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
  private Trigger AutoSourceTrigger = secondaryController.povUp();
  private Trigger AutoTrapTrigger = secondaryController.povDown();
  private Trigger Amptrigger = secondaryController.back();
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
  // climb triggers
  private Trigger climbTrigger = secondaryController.b().and(
      secondaryController.axisGreaterThan(1, Constants.controllerDeadband)
          .or(secondaryController.axisLessThan(1, -Constants.controllerDeadband))); // left joystick y moved while b held

          private AmpCommandGroup ampCommandGroup = new AmpCommandGroup(trapMechanismSubsystem, trapHeightPIDSubsystem, expelTrapTrigger);
  public RobotContainer() {
    swerve.setDefaultCommand(teleopSwerve);
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
    AutoSourceTrigger.whileTrue(intakeTrapNoteCommand);
    AutoTrapTrigger.whileTrue(autoTrapCommand);
    Amptrigger.whileTrue(ampCommandGroup);
    

    // shooter bindings
    aimShooterTrigger.whileTrue(aimShooterCommand);
    raiseShooterTrigger.onTrue(new InstantCommand(shooterHeightPIDSubsystem::increaseAngle));
    lowerShooterTrigger.onTrue(new InstantCommand(shooterHeightPIDSubsystem::decreaseAngle));
    spinShooterTrigger.whileTrue(spinShooterCommand);
    // intake bindings
    intakeTrigger.whileTrue(intakeCommandGroup);
    reverseIntakeTrigger.whileTrue(new StartEndCommand(intakeSubsystem::reverse, intakeSubsystem::stopAll, intakeSubsystem));
    intakeFeedTrigger.whileTrue(new StartEndCommand(intakeSubsystem::startAll, intakeSubsystem::stopAll, intakeSubsystem));
    shooterFeedTrigger.whileTrue(new StartEndCommand(intakeSubsystem::startShooterFeed, intakeSubsystem::stopAll, intakeSubsystem));
    // climb bindings
    climbTrigger.whileTrue(climbCommand);
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
