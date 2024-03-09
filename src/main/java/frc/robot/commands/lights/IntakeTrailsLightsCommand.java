// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lights;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LightConstants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.PowerSubsystem;

public class IntakeTrailsLightsCommand extends Command {
  LightsSubsystem lightsSubsystem;

  Color8Bit[] currentPattern;
  Color8Bit[] nextPattern;
  Timer timer = new Timer();

  /** Creates a new DisabledLightsCommand. */
  public IntakeTrailsLightsCommand(LightsSubsystem lightsSubsystem) {
    this.lightsSubsystem = lightsSubsystem;

    currentPattern = LightConstants.greenTrails;
    nextPattern = lightsSubsystem.rearrangePattern(LightConstants.greenTrails, LightConstants.forwardShiftIndices);

    addRequirements(lightsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lightsSubsystem.setPattern(currentPattern);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double t = timer.get()*LightConstants.rainbowSpeed;
    Color8Bit[] pattern = lightsSubsystem.interpolatePatterns(currentPattern, nextPattern, t);
    lightsSubsystem.setPattern(pattern);
    
    if (t >= 1.0) {
      currentPattern = nextPattern.clone();
      nextPattern = lightsSubsystem.rearrangePattern(nextPattern, LightConstants.forwardShiftIndices);
      timer.restart();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

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
