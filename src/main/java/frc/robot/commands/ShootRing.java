// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootRing extends Command {
  private final Shooter m_shooter;

  public ShootRing(Shooter sh) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = sh;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.testShoot(m_shooter.shootingVelocity);
    m_shooter.testAngle(m_shooter.shootingAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooter.atSpeed()) {
      m_shooter.indexRun(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
    m_shooter.indexStop();
    m_shooter.stopAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
