// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class OuttakeRun extends Command {
  private final Intake m_intake;
  /** Creates a new IntakeRun. */
  public OuttakeRun(Intake in) {
    m_intake = in;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.donutRelease();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.motorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; 
  }
}
