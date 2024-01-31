// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToNoteCommand extends ProfiledPIDCommand {
  /** Creates a new AlignToNoteCommand. */
  public AlignToNoteCommand(Swerve swerve, DoubleSubscriber targetHeadingSubscriber) {
    super(
        // The ProfiledPIDController used by the command
        SwerveConstants.alignPID,
        // This should return the measurement
        targetHeadingSubscriber::get,
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(Radians.zero(), RadiansPerSecond.zero()),
        // This uses the output
        (output, setpoint) -> {
          swerve.driveRobotCentric(MetersPerSecond.zero(), MetersPerSecond.zero(), RadiansPerSecond.of(output));
        },
        swerve);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
