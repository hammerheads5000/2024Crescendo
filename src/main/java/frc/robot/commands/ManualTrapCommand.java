// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
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
  public void initialize() {}

  @Override
  public void execute() 
  {
    if(Math.abs(controller.getRightY()) > Constants.SwerveConstants.controllerDeadband)
    {
      trapSub.disable();
      trapSub.raise(controller.getRightY());
    }
    else
    {
      trapSub.stop();
      if(!trapSub.isEnabled())
      {
        trapSub.setSetpoint(trapSub.getMeasurement());
        trapSub.enable();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
