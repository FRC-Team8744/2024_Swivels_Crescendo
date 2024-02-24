// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeRun extends Command {
  private final Intake m_intake;
  private final Shooter m_shooter;
  private final Index m_index;
  /** Creates a new IntakeRun. */
  public IntakeRun(Intake in, Shooter sh, Index ind) {
    m_intake = in;
    m_shooter = sh;
    m_index = ind;
    addRequirements(m_intake);    
    addRequirements(m_shooter);
    addRequirements(m_index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.donutGrab(-m_intake.intakeSpeed);
    m_index.indexRun(-m_index.indexSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.motorOff();
    m_index.indexStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_index.inputIR.get();
  }
}