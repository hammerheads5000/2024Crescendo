// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ActuatorSubsystem;

public class RobotContainer {
  Servo actuatorServo = new Servo(1); // PWM channel 1
  ActuatorSubsystem actuatorSubsystem = new ActuatorSubsystem(actuatorServo);
  ActuateCommand actuateCommand = new ActuateCommand(actuatorSubsystem);

  CommandXboxController controller = new CommandXboxController(0);
  Trigger actuateTrigger = controller.a();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    actuateTrigger.whileTrue(actuateCommand);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
