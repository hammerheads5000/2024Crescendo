// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends SubsystemBase {
  /** Creates a new LightsSubsystem. */
  AddressableLED ledstrip;
  AddressableLEDBuffer m_ledBuffer;
  public LightsSubsystem() 
  {
    ledstrip = new AddressableLED(0);
    ledstrip.setLength(m_ledBuffer.getLength());
    m_ledBuffer = new AddressableLEDBuffer(60);

    ledstrip.setData(m_ledBuffer);
    ledstrip.start();
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void SetBlue()
  {
    for (var i = 0; i < 60; i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 0, 255);
   }
   
   ledstrip.setData(m_ledBuffer);
  }
}
