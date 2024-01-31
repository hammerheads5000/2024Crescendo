// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToNoteCommand extends PIDCommand {
  /** Creates a new AlignToNoteCommand. */
  public AlignToNoteCommand(Swerve swerve, DoubleSubscriber targetHeadingSubscriber, BooleanSubscriber hasTargetSubscriber) {
    super(
        // The PIDController used by the command
        SwerveConstants.alignPID,
        // Measurement is the yaw to note
        targetHeadingSubscriber::get,
        // Setpoint at 0 meaning the goal is to align with note so angle between is 0
        0,
        // This uses the output
        output -> {
          if (!hasTargetSubscriber.get()) { return; } // do nothing if no targets
          swerve.driveRobotCentric(MetersPerSecond.zero(), MetersPerSecond.zero(), DegreesPerSecond.of(-output));
        },
        swerve);

    getController().enableContinuousInput(-180, 180); // not strictly necessary, but good practice
    getController().setTolerance(SwerveConstants.rotationalTolerance.in(Degrees));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
