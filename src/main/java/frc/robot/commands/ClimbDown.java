// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbDown extends Command {
  /** Creates a new ClimbDown. */
  private final Climber m_climber;
  private int sensorState;
  public ClimbDown(Climber cl) {
    m_climber = cl;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.climberDown(m_climber.climbSpeed);
    sensorState = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (sensorState == 0 && Math.abs(m_climber.climberVelocity()) > 0.5) sensorState = 1;
    if (sensorState == 1 && Math.abs(m_climber.climberVelocity()) < 0.5) return true;
    return false;
  }
}
