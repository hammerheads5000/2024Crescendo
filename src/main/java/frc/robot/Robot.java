// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.trapmechanism.TrapMechanismSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  TrapMechanismSubsystem trapMechanismSubsystem;
  boolean isClimbing = false;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    this.trapMechanismSubsystem = m_robotContainer.trapMechanismSubsystem;
    m_robotContainer.swerve.resetPose(new Pose2d()); // reset field relative
    
    // logging
    SmartDashboard.putData(CommandScheduler.getInstance());
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.updateValues();
  }

  @Override
  public void disabledInit() {
   m_robotContainer.disabledLightsCommand.schedule();
   trapMechanismSubsystem.contractActuator();
   if (isClimbing) {
    m_robotContainer.autoTrapCommand.schedule();
   }
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    m_robotContainer.disabledLightsCommand.cancel();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.shooterHeightPIDSubsystem.setSetpoint(m_robotContainer.shooterHeightPIDSubsystem.getMeasurement());
    m_robotContainer.shooterHeightPIDSubsystem.enable();
    trapMechanismSubsystem.extendActuator();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    isClimbing = m_robotContainer.autoTrapCommand.isScheduled();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
