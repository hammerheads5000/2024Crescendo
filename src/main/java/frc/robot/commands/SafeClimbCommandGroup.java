// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.trapmechanism.TrapHeightPIDSubsystem;
import frc.robot.subsystems.trapmechanism.TrapMechanismSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SafeClimbCommandGroup extends SequentialCommandGroup {
  /** Creates a new SafeClimbCommandGroup. */
  public SafeClimbCommandGroup(TrapHeightPIDSubsystem trapPIDSubsystem, ClimberSubsystem climberSub, TrapMechanismSubsystem trapMechanismSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new InstantCommand(trapPIDSubsystem::moveToHome),
    new InstantCommand(trapMechanismSub::contractActuator),
    new InstantCommand(() -> climberSub.climb(Constants.ClimberConstants.climbSpeed))
    );
  }
}
