// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class MoveOverNoteCommand extends Command {
  Swerve swerve;
  IntakeSubsystem intakeSubsystem;

  /** Creates a new MoveOverNoteCommand. */
  public MoveOverNoteCommand(Swerve swerve, IntakeSubsystem intakeSubsystem) {
    this.swerve = swerve;
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(swerve, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.driveRobotCentric(IntakeConstants.moveOverVelocity, MetersPerSecond.zero(), RadiansPerSecond.zero()); // drive forward
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.driveRobotCentric(MetersPerSecond.zero(), MetersPerSecond.zero(), RadiansPerSecond.zero()); // stop
    if (!interrupted) {
      LoggingConstants.hasNotePublisher.set(true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSubsystem.intakeLidarState();
  }
}
