// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.intake.AlignToNoteCommand;
import frc.robot.commands.intake.MoveOverNoteCommand;
import frc.robot.commands.lights.IntakeTrailsLightsCommand;
import frc.robot.commands.shooter.AimShooterCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.ShooterHeightPIDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpNoteAndShootCommand extends SequentialCommandGroup {
  /** Creates a new PickUpNoteAndShootCommand. */
  public PickUpNoteAndShootCommand(Swerve swerve, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ShooterHeightPIDSubsystem shooterHeightPIDSubsystem, LightsSubsystem lightsSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    AimShooterCommand aimShooterCommand = new AimShooterCommand(swerve, shooterHeightPIDSubsystem);
    addCommands(
      new InstantCommand(() -> shooterHeightPIDSubsystem.setTargetAngle(ShooterConstants.farAngle)),
      new InstantCommand(() -> lightsSubsystem.setSolidColor(LightConstants.ORANGE)),
      new AlignToNoteCommand(swerve, lightsSubsystem),
      new InstantCommand(intakeSubsystem::startAll),
      new MoveOverNoteCommand(swerve, intakeSubsystem),
      new InstantCommand(() -> intakeSubsystem.setFeedSpeed(IntakeConstants.slowFeedRate)),
      new IntakeTrailsLightsCommand(lightsSubsystem).until(intakeSubsystem::shooterLidarState),
      Commands.race( // and aimShooterCommand when shot
          aimShooterCommand,
          Commands.sequence(
              new InstantCommand(intakeSubsystem::stopAll),
              new InstantCommand(() -> lightsSubsystem.setSolidColor(LightConstants.GREEN)),
              new WaitUntilCommand(() -> shooterSubsystem.flywheelsAtSpeed()
                  && shooterHeightPIDSubsystem.getController().atSetpoint()
                  && aimShooterCommand.isAligned()), // wait until ready to shoot
              new InstantCommand(() -> lightsSubsystem.setSolidColor(LightConstants.PINK)),
              new StartEndCommand(intakeSubsystem::startShooterFeed, intakeSubsystem::stopAll, intakeSubsystem)
                  .until(() -> !intakeSubsystem.shooterLidarState()) // feed to shoot until note not detected
          )
      ),
      new InstantCommand(() -> lightsSubsystem.setSolidColor(LightConstants.BLANK))
    );
  }
}
