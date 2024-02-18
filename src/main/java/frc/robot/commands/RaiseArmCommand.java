// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapHeightPIDSubsystem;
import frc.robot.subsystems.TrapMechanismSubsystem;

public class RaiseArmCommand extends Command {
  /** Creates a new LowerArm. */
  TrapHeightPIDSubsystem trapSubsystem;

  public RaiseArmCommand(TrapHeightPIDSubsystem trapSubsystem) {
    this.trapSubsystem = trapSubsystem;
    addRequirements(trapSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trapSubsystem.disable(); // disable pid during manual control
    trapSubsystem.raise();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trapSubsystem.setSetpoint(trapSubsystem.getMeasurement()); // set setpoint to current position
    trapSubsystem.enable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
