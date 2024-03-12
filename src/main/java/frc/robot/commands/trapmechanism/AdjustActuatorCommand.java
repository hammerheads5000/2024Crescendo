// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trapmechanism;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.trapmechanism.TrapMechanismSubsystem;

public class AdjustActuatorCommand extends Command {
  TrapMechanismSubsystem trapMechanismSubsystem;
  /** Creates a new AdjustActuatorCommand. */
  public AdjustActuatorCommand(TrapMechanismSubsystem trapMechanismSubsystem) {
    this.trapMechanismSubsystem = trapMechanismSubsystem;
    addRequirements(trapMechanismSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trapMechanismSubsystem.reverse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    trapMechanismSubsystem.stopRollers();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(trapMechanismSubsystem.isNoteDetected()){
      return false;
    }else{
      return true;
    }
  }
}
