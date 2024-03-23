// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LightConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.commands.lights.IntakeTrailsLightsCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightsSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FinishIntakeCommand extends SequentialCommandGroup {
  /** Creates a new ManualIntakeCommand. */
  public FinishIntakeCommand(IntakeSubsystem intakeSubsystem, LightsSubsystem lightsSubsystem) {
    addCommands(
      new IntakeTrailsLightsCommand(lightsSubsystem),
      new InstantCommand(intakeSubsystem::startAll),
      new InstantCommand(() -> intakeSubsystem.setFeedSpeed(IntakeConstants.slowFeedRate)),
      new WaitUntilCommand(intakeSubsystem::shooterLidarState),
      new InstantCommand(intakeSubsystem::stopAll),
      new InstantCommand(() -> lightsSubsystem.setSolidColor(LightConstants.BLANK))
    );
  }
}
