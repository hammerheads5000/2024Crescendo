// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private final TalonFX topMotor = new TalonFX(24);//invert
  private final TalonFX bottomMotor = new TalonFX(20);

  private final ShooterSubsystem shooterSub = new ShooterSubsystem(topMotor, bottomMotor);

  private final CommandXboxController controller = new CommandXboxController(0);

  private final Trigger onOffTrigger = controller.b();
  private final Trigger increaseTopTrigger = controller.y();
  private final Trigger decreaseTopTrigger = controller.a();
  private final Trigger increaseBottomTrigger = controller.povUp();
  private final Trigger decreaseBottomTrigger = controller.povDown();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    onOffTrigger.onTrue(new InstantCommand(() -> shooterSub.togglePower()));
    increaseTopTrigger.onTrue(new InstantCommand(() -> shooterSub.changeTopSpeed(ShooterConstants.speedIncrease)));
    decreaseTopTrigger.onTrue(new InstantCommand(() -> shooterSub.changeTopSpeed(-ShooterConstants.speedIncrease)));

    increaseBottomTrigger.onTrue(new InstantCommand(() -> shooterSub.changeBottomSpeed(ShooterConstants.speedIncrease)));
    decreaseBottomTrigger.onTrue(new InstantCommand(() -> shooterSub.changeBottomSpeed(-ShooterConstants.speedIncrease)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
