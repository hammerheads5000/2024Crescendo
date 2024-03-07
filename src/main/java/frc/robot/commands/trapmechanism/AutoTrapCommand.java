// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trapmechanism;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.trapmechanism.TrapHeightPIDSubsystem;
import frc.robot.subsystems.trapmechanism.TrapMechanismSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTrapCommand extends SequentialCommandGroup {
  /** Creates a new AutoTrapCommand. */
  public AutoTrapCommand(TrapHeightPIDSubsystem trapPIDSubsystem, TrapMechanismSubsystem trapSubsystem, ClimberSubsystem climbSubsystem, LightsSubsystem lightsSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> lightsSubsystem.setSolidColor(Constants.LightConstants.RED)),
      new InstantCommand(trapSubsystem::contractActuator),
      new InstantCommand(trapPIDSubsystem::moveToTrap),
      new WaitCommand(3),
      new InstantCommand(() -> climbSubsystem.climb(ClimberConstants.climbSpeed)),
      new WaitUntilCommand(climbSubsystem::reachedClimbLimit),
      new InstantCommand(() -> lightsSubsystem.setSolidColor(Constants.LightConstants.GREEN)),
      new WaitCommand(3),
      new InstantCommand(trapSubsystem::forward)
    );
  }
}
