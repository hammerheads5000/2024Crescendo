// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCommandGroup extends SequentialCommandGroup {
  public static boolean isFinished;
  /** Creates a new IntakeCommandGroup. */
  public IntakeCommandGroup(Swerve swerve, IntakeSubsystem intakeSubsystem, LightsSubsystem lightsSubsystem) {    
    addCommands(
      new InstantCommand(() -> lightsSubsystem.setSolidColor(Constants.LightConstants.RED)),
      new AlignToNoteCommand(swerve, lightsSubsystem),
      new InstantCommand(intakeSubsystem::startAll),
      new MoveOverNoteCommand(swerve, intakeSubsystem),
      new InstantCommand(() -> lightsSubsystem.setSolidColor(Constants.LightConstants.GREEN)),
      new InstantCommand(() -> intakeSubsystem.setFeedSpeed(Constants.IntakeConstants.slowFeedRate)),
      new WaitUntilCommand(intakeSubsystem::shooterLidarState),
      new InstantCommand(intakeSubsystem::stopAll)
    );
  }
}
