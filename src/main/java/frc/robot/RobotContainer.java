// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  TalonSRX rollers = new TalonSRX(12);
  Servo linearActuator = new Servo(2);

  CommandXboxController controller = new CommandXboxController(0);
  Trigger intakeTrigger = controller.rightBumper();
  Trigger expelTrigger = controller.leftBumper();
  Trigger autoTrigger = controller.a();
  Trigger extendTrigger = controller.x();
  Trigger contractTrigger = controller.b();
  public DigitalInput lidar = new DigitalInput(4);

  
  Command intakeCommand = new FunctionalCommand(
    () -> rollers.set(TalonSRXControlMode.PercentOutput, -0.5), 
    () -> {}, 
    (interrupted) -> {}, 
    () -> !lidar.get(), 
    new Subsystem[0]
  ).andThen(new WaitCommand(0.3), new InstantCommand(rollers::neutralOutput));

  public RobotContainer() {
    linearActuator.setBoundsMicroseconds(2000, 0, 1500, 0, 1000);
    configureBindings();
  }

  private void configureBindings() {
    intakeTrigger.whileTrue(new StartEndCommand(() -> rollers.set(TalonSRXControlMode.PercentOutput, 0.5), () -> rollers.neutralOutput(), new Subsystem[0]));
    expelTrigger.whileTrue(new StartEndCommand(() -> rollers.set(TalonSRXControlMode.PercentOutput, -0.5), () -> rollers.neutralOutput(), new Subsystem[0]));
    autoTrigger.whileTrue(intakeCommand);
    extendTrigger.onTrue(new InstantCommand(() -> linearActuator.set(1.0)));
    contractTrigger.onTrue(new InstantCommand(() -> linearActuator.set(0.0)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
