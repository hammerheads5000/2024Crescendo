// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trapmechanism;

import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.TrapConstants;
import frc.robot.subsystems.trapmechanism.TrapMechanismSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.trapmechanism.TrapHeightPIDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeTrapNoteCommandGroup extends SequentialCommandGroup {
  /** Creates a new IntakeTrapNoteCommandGroup. */
  public IntakeTrapNoteCommandGroup(TrapMechanismSubsystem trapSubsystem, TrapHeightPIDSubsystem trapPIDSubsystem, LightsSubsystem lightsSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(trapPIDSubsystem::enable),
      new InstantCommand(() -> lightsSubsystem.setSolidColor(Constants.LightConstants.PINK)),
      new InstantCommand(trapPIDSubsystem::moveToSource),
      new InstantCommand(trapSubsystem::extendActuator),
      new InstantCommand(trapSubsystem::forward),
      new WaitUntilCommand(trapSubsystem::isNoteDetected),
      new WaitCommand(TrapConstants.intakeDelay.in(Seconds)),
      new InstantCommand(trapSubsystem::stopRollers)
      //new InstantCommand(() -> lightsSubsystem.setSolidColor(Constants.LightConstants.GREEN))
    );
  }
}
