// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.IntakeCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCommandGroup extends SequentialCommandGroup {
  /** Creates a new AutoCommandGroup. */
  public AutoCommandGroup(Command Path1, Command Path2, Swerve swerve, IntakeSubsystem intakeSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    ChassisSpeeds chassisSpeed = new ChassisSpeeds(0,0,0);
    addCommands(
    Path1,
    new InstantCommand(swerve::justStop),
    new WaitCommand(2),
    Path2,
     new InstantCommand(swerve::justStop)
    );
  }
}
