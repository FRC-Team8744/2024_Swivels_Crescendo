// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;

// import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision2;

public class LockOnShooter extends Command {
  private final Pivot m_pivot;
  private final Vision2 m_vision;
  private final DriveSubsystem m_drive;
  private final Shooter m_shooter;
  private final LEDS m_leds;
  private boolean toggle;
  /** Creates a new LockOnShooter. */
  public LockOnShooter(Pivot pi, Vision2 vis, DriveSubsystem dr, Shooter sh, LEDS le) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pivot = pi;
    m_vision = vis;
    m_drive = dr;
    m_shooter = sh;
    m_leds = le;
    addRequirements(m_pivot);
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.testShoot(3500);
    m_leds.ledOn(0, 0, 255);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_leds.rainbow(19, 33);

    var alliance = DriverStation.getAlliance();

    if (alliance.get() == DriverStation.Alliance.Red) {
      m_pivot.visionShootAngle = Math.toDegrees(Math.atan(m_vision.getTargetHeight() / Math.abs(m_drive.getEstimatedPoseHyp()))) + Math.abs(m_drive.getEstimatedPoseHyp()) -1; 
    }

    else {
      m_pivot.visionShootAngle = Math.toDegrees(Math.atan((m_vision.getTargetHeight() - Units.inchesToMeters(9.481)) / (m_drive.getEstimatedPoseHyp() - Units.inchesToMeters(1.15104)))) + Math.abs(m_drive.getEstimatedPoseHyp()) + 1;
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

  public void toggle() {
    toggle = !toggle;
    if (toggle) {
      this.schedule();
    }
    // SmartDashboard.putBoolean("Toggle", toggle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.stopAngle();
    m_shooter.stopShooter();
    m_leds.ledOn(0, 0, 255);
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !toggle;
  }

  public void reset() {
    toggle = false;
  }
}
