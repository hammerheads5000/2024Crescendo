// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trapmechanism;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.trapmechanism.TrapMechanismSubsystem;

public class ManualRollerCommand extends Command {
  /** Creates a new ManualRollerCommand. */
  TrapMechanismSubsystem trapMechanismSubsystem;
  CommandXboxController controller;
  public ManualRollerCommand(TrapMechanismSubsystem trapMechanismSubsystem, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.trapMechanismSubsystem = trapMechanismSubsystem;
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    trapMechanismSubsystem.moveManual(controller.getLeftY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
