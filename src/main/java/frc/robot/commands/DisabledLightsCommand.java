// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LightConstants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.PowerSubsystem;

public class DisabledLightsCommand extends Command {
  LightsSubsystem lightsSubsystem;
  AprilTagSubsystem aprilTagSubsystem;
  PowerSubsystem powerSubsystem;

  /** Creates a new DisabledLightsCommand. */
  public DisabledLightsCommand(LightsSubsystem lightsSubsystem, AprilTagSubsystem aprilTagSubsystem, PowerSubsystem powerSubsystem) {
    this.lightsSubsystem = lightsSubsystem;
    this.aprilTagSubsystem = aprilTagSubsystem;
    this.powerSubsystem = powerSubsystem;

    addRequirements(lightsSubsystem, aprilTagSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lightsSubsystem.setPattern(LightConstants.rainbow);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (powerSubsystem.getVoltage() < Constants.lowStartVoltage) {
      lightsSubsystem.setSolidColor(LightConstants.RED);
    }
    else if (aprilTagSubsystem.hasAprilTag()) {
      lightsSubsystem.setSolidColor(LightConstants.GREEN);
    }
    else {
      lightsSubsystem.setPattern(LightConstants.rainbow);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Run when disabled
  @Override
  public boolean runsWhenDisabled() {
      return true;
  }
}