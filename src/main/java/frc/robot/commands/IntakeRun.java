// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeRun extends Command {
  private final Intake m_intake;
  private final Shooter m_shooter;
  /** Creates a new IntakeRun. */
  public IntakeRun(Intake in, Shooter sh) {
    m_intake = in;
    m_shooter = sh;
    addRequirements(m_intake);    
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double speedSetting = -.3;
    m_intake.donutGrab(speedSetting);
    m_shooter.indexRun(speedSetting * 3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.motorOff();
    m_shooter.indexStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_intake.inputIR.get(); 
  }
}
