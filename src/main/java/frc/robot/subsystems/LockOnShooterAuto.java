// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LockOnShooterAuto extends SubsystemBase {
  private Pivot m_pivot;
  private Vision2 m_vision;
  private DriveSubsystem m_drive;
  private Shooter m_shooter;
  private boolean toggle;
  /** Creates a new LockOnShooterAuto. */
  public LockOnShooterAuto(Pivot pi, Vision2 vis, DriveSubsystem dr, Shooter sh) {
    m_pivot = pi;
    m_vision = vis;
    m_drive = dr;
    m_shooter = sh;
  }

  @Override
  public void periodic() {
    if (toggle) {
      var alliance = DriverStation.getAlliance();

      if (alliance.get() == DriverStation.Alliance.Red) {
        m_pivot.visionShootAngle = Math.toDegrees(Math.atan(m_vision.getTargetHeight() / Math.abs(m_drive.getEstimatedPoseHyp()))) + Math.abs(m_drive.getEstimatedPoseHyp()) -1; 
      }

      else {
        m_pivot.visionShootAngle = Math.toDegrees(Math.atan(m_vision.getTargetHeight() / m_drive.getEstimatedPoseHyp())) + (m_drive.getEstimatedPoseHyp() -1);
      }

      if (m_pivot.visionShootAngle <= Pivot.minimumAngle) {
        m_pivot.visionShootAngle = Pivot.minimumAngle;
      }

      m_pivot.testAngle(m_pivot.visionShootAngle);

      m_shooter.visionShootVelocity = 2500 + (390.625 * m_drive.getEstimatedPoseHyp());

      if (m_shooter.visionShootVelocity >= (m_shooter.visionShootVelocityLimit / 12) * Constants.PDH.getVoltage())  {
        m_shooter.visionShootVelocity = (m_shooter.visionShootVelocityLimit / 12) * Constants.PDH.getVoltage();
      }
    }
  }

  public Command toggle() {
    return Commands.runOnce(() -> {
      toggle = !toggle;
      if (toggle) {
        m_shooter.testShoot(3500);
      }
    });
  }

  public void reset() {
    toggle = false;
    m_pivot.stopAngle();
  }

  public void enable() {
    toggle = true;
  }
}
