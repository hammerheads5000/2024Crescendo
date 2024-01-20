// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX topMotor;
  private TalonFX bottomMotor;
  
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("shooter");
  private DoubleEntry topSpeedEntry = table.getDoubleTopic("Top Motor Speed").getEntry(0);
  private DoubleEntry bottomSpeedEntry = table.getDoubleTopic("Bottom Motor Speed").getEntry(0);

  private boolean isOn = false;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(TalonFX topMotor, TalonFX bottomMotor) {
    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;
  }
  
  // in rotations per second
  public void changeTopSpeed(double amt) {
    double current = topSpeedEntry.get(); 
    topSpeedEntry.set(current+amt);
  }

  // in rotations per second
  public void changeBottomSpeed(double amt) {
    double current = bottomSpeedEntry.get(); 
    bottomSpeedEntry.set(current+amt);
  }

  public void togglePower() {
    isOn = !isOn;
    
    if (isOn) {
      topMotor.setControl(new VelocityDutyCycle(topSpeedEntry.get()));
      bottomMotor.setControl(new VelocityDutyCycle(bottomSpeedEntry.get()));
    }
    else {
      topMotor.set(0.0);
      bottomMotor.set(0.0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
