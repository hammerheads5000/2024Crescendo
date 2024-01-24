// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase {
  private SwerveDrivetrain drivetrain;
  private SwerveRequest.FieldCentric driveRequest;

  /** Creates a new Swerve. */
  public Swerve() {
    drivetrain = SwerveConstants.drivetrain;

    drivetrain.configNeutralMode(SwerveConstants.driveNeutralMode);
    for (int i = 0; i < 4; i++) {
      TalonFXConfigurator driveMotorConfig = drivetrain.getModule(i).getDriveMotor().getConfigurator();
      driveMotorConfig.apply(SwerveConstants.driveCurrentLimits);
      driveMotorConfig.apply(SwerveConstants.closedLoopRampsConfig);
      
      TalonFXConfigurator angleMotorConfig = drivetrain.getModule(i).getSteerMotor().getConfigurator();
      angleMotorConfig.apply(SwerveConstants.closedLoopRampsConfig);
    }
    
    // setup drive request
    driveRequest = new SwerveRequest.FieldCentric()
      .withDeadband(SwerveConstants.velocityDeadband.in(MetersPerSecond))
      .withRotationalDeadband(SwerveConstants.rotationDeadband.in(RadiansPerSecond))
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagic);
  }

  /**
   * Drive robot with respect to field
   * @param xVel forward velocity in meters / sec
   * @param yVel left velocity in meters / sec
   * @param rot angular velocity counterclockwise in radians / sec
   */
  public void drive(Measure<Velocity<Distance>> xVel, Measure<Velocity<Distance>> yVel, Measure<Velocity<Angle>> rot) {

    drivetrain.setControl(
      driveRequest.withVelocityX(xVel.in(MetersPerSecond))
        .withVelocityY(yVel.in(MetersPerSecond))
        .withRotationalRate(rot.in(RadiansPerSecond))
    );
  }

  public void zeroFOC() {
    drivetrain.seedFieldRelative();
  }

  @Override
  public void periodic() { }
}
