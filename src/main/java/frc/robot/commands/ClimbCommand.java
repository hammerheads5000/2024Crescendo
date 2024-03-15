// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.shooter.ShooterHeightPIDSubsystem;
import frc.robot.subsystems.trapmechanism.TrapHeightPIDSubsystem;

public class ClimbCommand extends Command {
  ClimberSubsystem climbSubsystem;
  CommandXboxController controller;
  ShooterHeightPIDSubsystem shooterSubsystem;
  TrapHeightPIDSubsystem trapPIDSubsystem;
  /** Creates a new ClimbCommand. */
  public ClimbCommand(ClimberSubsystem climbSubsystem, CommandXboxController controller, ShooterHeightPIDSubsystem shooterSubsystem, TrapHeightPIDSubsystem trapPIDSubsystem) {
    this.controller = controller;
    this.climbSubsystem = climbSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(climbSubsystem, shooterSubsystem);
    this.trapPIDSubsystem = trapPIDSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.disable();
    shooterSubsystem.coast();
    //trapPIDSubsystem.moveToHome();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      climbSubsystem.climb(controller.getRightY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    climbSubsystem.climb(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
