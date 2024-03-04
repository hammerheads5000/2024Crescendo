// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;


public class SpinShooterCommand extends Command {
  ShooterSubsystem shooterSubsystem;
  /** Creates a new SpinShooterCommand. */
  public SpinShooterCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Flywheels At Full Speed", shooterSubsystem.flywheelsAtSpeed());
    SmartDashboard.putBoolean("Flywheels At Low Speed", shooterSubsystem.flywheelsAtCloseSpeed());
    SmartDashboard.putNumber("Flywheel RPM", shooterSubsystem.getFlywheelSpeed().in(Units.RPM));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
