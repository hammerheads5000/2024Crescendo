// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class LowerArmCommand extends Command {
  DoubleSolenoid armPneumatic;
  /** Creates a new LowerArm. */
  public LowerArmCommand(DoubleSolenoid doubleSolenoid) {
    armPneumatic = doubleSolenoid;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armPneumatic.set(kForward);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armPneumatic.set(kReverse);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return IntakeCommandGroup.isFinished;
  }
}
