// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.TrapConstants;
import frc.robot.subsystems.TrapHeightPIDSubsystem;

public class ManualTrapCommand extends Command {
  CommandXboxController controller;
  TrapHeightPIDSubsystem trapSub;
  public ManualTrapCommand(CommandXboxController controller, TrapHeightPIDSubsystem trapSub) {
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
    if ((Inches.of(trapSub.getMeasurement()).gte(TrapConstants.maxHeight) && -controller.getRightY() > 0)
        || (trapSub.getMeasurement() <= 0 && -controller.getRightY() < 0))
      return;

    trapSub.raise(-controller.getRightY());
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
