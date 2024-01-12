// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;

public class Swerve extends SubsystemBase {
  private SwerveDrivetrain drivetrain;
  private SwerveRequest.FieldCentric driveRequest;

  /** Creates a new Swerve. */
  public Swerve() {
    drivetrain = Constants.Swerve.drivetrain;

    drivetrain.configNeutralMode(Constants.Swerve.driveNeutralMode);
    for (int i = 0; i < 4; i++) {
      TalonFXConfigurator driveMotorConfig = drivetrain.getModule(i).getDriveMotor().getConfigurator();
      driveMotorConfig.apply(Constants.Swerve.driveCurrentLimits);
      driveMotorConfig.apply(Constants.Swerve.closedLoopRampsConfig);
      
      TalonFXConfigurator angleMotorConfig = drivetrain.getModule(i).getSteerMotor().getConfigurator();
      angleMotorConfig.apply(Constants.Swerve.closedLoopRampsConfig);
    }
    driveRequest = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.Swerve.velocityDeadband)
      .withRotationalDeadband(Constants.Swerve.rotationDeadband)
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);
  }

  /**
   * Drive robot with respect to field
   * @param xVel x velocity in meters / sec
   * @param yVel y velocity in meters / sec
   * @param rot angular velocity in radians / sec
   */
  public void drive(double xVel, double yVel, double rot) {
    
    drivetrain.setControl(
      driveRequest.withVelocityX(xVel)
        .withVelocityY(yVel)
        .withRotationalRate(rot)
    );
  }

  @Override
  public void periodic() {}
}
