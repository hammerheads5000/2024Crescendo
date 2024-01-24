// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ActuatorSubsystem extends SubsystemBase {
  private Servo servo;

  /** Creates a new ActuatorSubsystem. */
  public ActuatorSubsystem(Servo servo) {
    this.servo = servo;
    servo.setBoundsMicroseconds(2000, 0, 1500, 0, 1000);
  }

  public void extend() {
    servo.set(1.0);
  }

  public void contract() {
    servo.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
