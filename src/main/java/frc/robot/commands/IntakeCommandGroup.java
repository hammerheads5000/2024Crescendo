// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCommandGroup extends SequentialCommandGroup {
  public static boolean isFinished;
  /** Creates a new IntakeCommandGroup. */
  public IntakeCommandGroup(Swerve swerve, IntakeSubsystem intakeSubsystem) {    
    addCommands(
     // new AlignToNoteCommand(swerve),
      new InstantCommand(() -> intakeSubsystem.startAll()), // start intake
      new WaitUntilCommand(intakeSubsystem::intakeLidarState),
      //new MoveOverNoteCommand(swerve),
      new InstantCommand(() -> intakeSubsystem.report()),
      new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(Constants.IntakeConstants.slowFeedRate)),
      new WaitUntilCommand(intakeSubsystem::shooterLidarState),
      new InstantCommand(() -> intakeSubsystem.stopFeeding())
    );
  }
}
