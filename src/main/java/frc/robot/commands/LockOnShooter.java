// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Vision2;

public class LockOnShooter extends Command {
  private final Pivot m_pivot;
  private final Vision2 m_vision;
  private boolean toggle;
  /** Creates a new LockOnShooter. */
  public LockOnShooter(Pivot pi, Vision2 vis) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pivot = pi;
    m_vision = vis;
    addRequirements(m_vision);
    addRequirements(m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_vision.getTargetDistance() > 8.23) {m_pivot.visionShootAngle = Math.toDegrees(Math.atan(m_vision.getTargetVertAngle() / Math.abs(m_vision.getTargetDistance() - 16.459))) + Math.abs(m_vision.getTargetDistance() - 16.459) * 2/3 -1; }

    else {m_pivot.visionShootAngle = Math.toDegrees(Math.atan(m_vision.getTargetVertAngle() / m_vision.getTargetDistance())) + (m_vision.getTargetDistance() * 2/3 -1);}

    m_pivot.testAngle(m_pivot.visionShootAngle);
  }

  public void toggle() {
    toggle = !toggle;
    if (toggle) {
      this.schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.stopAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !toggle;
  }
}
