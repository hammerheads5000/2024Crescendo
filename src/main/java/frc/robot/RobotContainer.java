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
import frc.robot.commands.AimShooterCommand;
import frc.robot.commands.ExpelTrapNoteCommand;
import frc.robot.commands.IntakeCommandGroup;
import frc.robot.commands.LowerArmCommand;
import frc.robot.commands.RaiseArmCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TrapMechanismSubsystem;

public class RobotContainer {
  private CommandXboxController controller = new CommandXboxController(0);

  // subsystems
  private Swerve swerve = new Swerve();
  private AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem(); // DO NOT REMOVE. Need periodic
  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private TrapMechanismSubsystem trapMechanismSubsystem = new TrapMechanismSubsystem();
  private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  
  // commands
  private AimShooterCommand aimShooterCommand = new AimShooterCommand(swerve, controller, shooterSubsystem);
  private TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, controller);
  private IntakeCommandGroup intakeCommandGroup = new IntakeCommandGroup(swerve, intakeSubsystem);
  private ExpelTrapNoteCommand expelTrapNoteCommand = new ExpelTrapNoteCommand(trapMechanismSubsystem);
  private LowerArmCommand lowerArmCommand = new LowerArmCommand(trapMechanismSubsystem);
  private RaiseArmCommand raiseArmCommand = new RaiseArmCommand(trapMechanismSubsystem);
  
  // autos
  private PathPlannerAuto ampAuto = new PathPlannerAuto("Amp");

  // triggers
  private Trigger zeroTrigger = controller.y();
  private Trigger ampTrigger = controller.a();
  private Trigger aimShooterTrigger = controller.leftBumper();
  private Trigger intakeTrigger = controller.rightBumper();
  private Trigger raiseTrapTrigger = controller.povUp();
  private Trigger lowerTrapTrigger = controller.povDown();
  private Trigger feedTrapTrigger = controller.povRight();
  private Trigger expelTrapTrigger = controller.povLeft();


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
    ampTrigger.whileTrue(ampAuto);
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
      
      NamedCommands.registerCommand("Raise To Amp", new InstantCommand());
      NamedCommands.registerCommand("Flip Trap Down", new InstantCommand(() -> trapMechanismSubsystem.extendActuator()));
      NamedCommands.registerCommand("Expel Trap Note", expelTrapNoteCommand);
      NamedCommands.registerCommand("Lower Trap Arm", lowerArmCommand);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
