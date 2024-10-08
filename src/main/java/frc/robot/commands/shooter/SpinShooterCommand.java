// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;


public class SpinShooterCommand extends Command {
  ShooterSubsystem shooterSubsystem;
  LightsSubsystem lightsSubsystem;
  CommandXboxController controller;
  
  /** Creates a new SpinShooterCommand. */
  public SpinShooterCommand(ShooterSubsystem shooterSubsystem, LightsSubsystem lightsSubsystem, CommandXboxController controller) {
    this.shooterSubsystem = shooterSubsystem;
    this.lightsSubsystem = lightsSubsystem;
    this.controller = controller;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.spin();
    LoggingConstants.shooterAtSpeedPublisher.set(shooterSubsystem.flywheelsAtSpeed());

    controller.getHID().setRumble(RumbleType.kBothRumble, shooterSubsystem.flywheelsAtSpeed() ? Constants.controllerRumble : 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    LoggingConstants.shooterAtSpeedPublisher.set(false);
    controller.getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
