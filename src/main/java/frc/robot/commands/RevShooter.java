// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

public class RevShooter extends Command {
  /** Creates a new RevShooter. */
  private final Shooter m_shooter;
  private final Index m_index;
  public RevShooter(Shooter sh, Index ind) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = sh;
    m_index = ind;
    addRequirements(m_shooter);
    addRequirements(m_index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_index.inputIR.get() == false) {
    m_shooter.testShoot(m_shooter.shootingVelocity);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
