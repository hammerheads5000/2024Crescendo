// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.intake.AlignToNoteCommand;
import frc.robot.commands.intake.MoveOverNoteCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.ShooterHeightPIDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpNoteAndShootCommand extends SequentialCommandGroup {
  /** Creates a new PickUpNoteAndShootCommand. */
  public PickUpNoteAndShootCommand(Swerve swerve, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ShooterHeightPIDSubsystem shooterHeightPIDSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    AimShooterCommand aimShooterCommand = new AimShooterCommand(swerve, shooterHeightPIDSubsystem);
    addCommands(
      new AlignToNoteCommand(swerve),
      new InstantCommand(intakeSubsystem::startAll),
      new MoveOverNoteCommand(swerve, intakeSubsystem),
      Commands.race( // and aimShooterCommand when shot
          aimShooterCommand,
          Commands.sequence(
              new WaitUntilCommand(intakeSubsystem::shooterLidarState), // wait for note loaded
              new InstantCommand(intakeSubsystem::stopFeeding),
              new WaitUntilCommand(() -> shooterSubsystem.flywheelsAtSpeed()
                  && shooterHeightPIDSubsystem.getController().atSetpoint()
                  && aimShooterCommand.isAligned()), // wait until ready to shoot
              new StartEndCommand(intakeSubsystem::startShooterFeed, intakeSubsystem::stopFeeding, intakeSubsystem)
                  .until(() -> !intakeSubsystem.shooterLidarState()) // feed to shoot until note not detected
          )
      )
    );
  }
}
