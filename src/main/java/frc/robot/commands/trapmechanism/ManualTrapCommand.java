// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trapmechanism;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.TrapConstants;
import frc.robot.subsystems.trapmechanism.TrapHeightPIDSubsystem;

public class ManualTrapCommand extends Command {
  CommandJoystick controller;
  TrapHeightPIDSubsystem trapSub;
  public ManualTrapCommand(CommandJoystick controller, TrapHeightPIDSubsystem trapSub) {
    this.controller = controller;
    this.trapSub = trapSub;
    addRequirements(trapSub);
  }

  @Override
  public void initialize() {
    trapSub.disable();
  }

  @Override
  public void execute() 
  {
    // check for endstops
    if ((Inches.of(trapSub.getMeasurement()).gte(TrapConstants.maxHeight) && -controller.getX() > 0)
        || (trapSub.getMeasurement() <= 0 && -controller.getX() < 0))
      return;

    trapSub.raise(-controller.getX());
  }

  @Override
  public void end(boolean interrupted) {
    trapSub.stop();
    trapSub.setSetpoint(trapSub.getMeasurement());
    trapSub.enable();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
