// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  TalonFX climbMotor;
  DigitalInput limitLidarSensor;
  DigitalInput limitLidarSensor2;
  DigitalInput slowLidarSensor;
  BooleanPublisher climberDownPublisher = Constants.LoggingConstants.climberDownPublisher;
  DoublePublisher climberSpeedPublisher = Constants.LoggingConstants.climberSpeedPublisher;
  LightsSubsystem lightsSubsystem;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(LightsSubsystem lightsSubsystem) {
    climbMotor = ClimberConstants.climberMotor;
    climbMotor.getConfigurator().apply(new MotorOutputConfigs()
      .withInverted(ClimberConstants.climberInverted)
      .withNeutralMode(NeutralModeValue.Brake));
    this.lightsSubsystem = lightsSubsystem;
    limitLidarSensor = ClimberConstants.limitLidarSensor;
    limitLidarSensor2 = ClimberConstants.limitLidarSensor2;
    slowLidarSensor = ClimberConstants.slowLidarSensor;
  }

  /**
   * Move the climber
   * @param speed speed to climb at. positive is climbing
   */
  public void climb(double speed) {
    //if (reachedClimbLimit() && speed > 0) return;
    climbMotor.set(ClimberConstants.climbSpeed * speed);
    climberSpeedPublisher.set(ClimberConstants.climbSpeed * speed);
  }

  public boolean reachedClimbLimit() {
    return !limitLidarSensor.get() || !limitLidarSensor2.get();
  }
  public boolean reachedSlowLimit(){
    return !slowLidarSensor.get();
  }
  
  public void stopMotor()
  {
    climbMotor.stopMotor();
  }
  
  public int getStageAutoNumber() {
    Alliance alliance = DriverStation.getAlliance().get();
    int driverStation = DriverStation.getLocation().getAsInt();

    if (alliance.equals(Alliance.Blue) && driverStation == 1) return 1;
    if (alliance.equals(Alliance.Red) && driverStation == 3) return 1;

    return 2;
  }
  
  @Override
  public void periodic() {
    climberDownPublisher.set(reachedClimbLimit());
    if (reachedClimbLimit()) {
      climbMotor.stopMotor();
    }
  }
}
