// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCommandGroup extends SequentialCommandGroup {
  /** Creates a new IntakeCommandGroup. */
  public IntakeCommandGroup(Swerve swerve) {
    DoubleSubscriber noteTargetYawSubscriber = VisionConstants.noteYawTopic.subscribe(0.0);
    
    addCommands(
      new AlignToNoteCommand(swerve, noteTargetYawSubscriber),
      //startIntake,
      new MoveOverNoteCommand(swerve)//,
      //loadNote
    );
  }
}
