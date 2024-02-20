// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  I2C.Port i2c = I2C.Port.kOnboard;
  public ColorSensorV3 colorSensor = new ColorSensorV3(i2c);
  public ColorMatch colorMatch = new ColorMatch();
  SuppliedValueWidget<Boolean> wiget;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    colorMatch.addColorMatch(new Color("#206080"));
    wiget = Shuffleboard.getTab("SmartDashboard")
      .addBoolean("Color match", () -> true)
      .withWidget(BuiltInWidgets.kBooleanBox);
    colorSensor.configureColorSensor(ColorSensorResolution.kColorSensorRes16bit, ColorSensorMeasurementRate.kColorRate50ms, GainFactor.kGain1x);
    colorMatch.setConfidenceThreshold(0.93);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putString("Color (r,g,b)", colorSensor.getColor().toString());
    ColorMatchResult colorMatchResult = colorMatch.matchColor(colorSensor.getColor());
    String result = "None";
    if (colorMatchResult != null) {
      result = colorMatchResult.color.toString() + " " + colorMatchResult.confidence;
    }
    SmartDashboard.putString("Color match", result);
    wiget.withProperties(Map.of("Color when true", colorSensor.getColor().toHexString()));
    SmartDashboard.putNumber("Blue", colorSensor.getColor().blue);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

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
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
