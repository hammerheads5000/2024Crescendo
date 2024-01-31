// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Swerve;

public class MoveOverNoteCommand extends Command {
  DigitalInput irSensor = new DigitalInput(IntakeConstants.lidarSensorChannel);
  Swerve swerve;

  /** Creates a new MoveOverNoteCommand. */
  public MoveOverNoteCommand(Swerve swerve) {
    this.swerve = swerve;

    addRequirements(swerve);
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return irSensor.get();
  }
}
