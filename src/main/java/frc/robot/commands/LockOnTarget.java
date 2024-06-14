// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.subsystems.DriveSubsystem;

public class LockOnTarget extends Command {
  private final DriveSubsystem m_drive;
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private Pose2d targetPose = aprilTagFieldLayout.getTagPose(7).get().toPose2d();
  private PIDController m_turnCtrl = new PIDController(0.02, 0, 0);
  private double goalAngle;
  private double heading;
  private double m_output;

  public LockOnTarget(DriveSubsystem dr) {
    m_drive = dr;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turnCtrl.enableContinuousInput(-180, 180);
    m_turnCtrl.setTolerance(1.0);
    m_turnCtrl.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    heading = m_drive.getEstimatedPose().getRotation().getDegrees();

    double distanceToTargetX = Math.abs(m_drive.getEstimatedPose().getX() - targetPose.getX());
    double distanceToTargetY = Math.abs(m_drive.getEstimatedPose().getY() - targetPose.getY());

    if (m_drive.getEstimatedPose().getY() > targetPose.getY()) {
      goalAngle = Math.atan(distanceToTargetX / distanceToTargetY) - heading;
    } else {
      goalAngle = Math.atan(distanceToTargetX / distanceToTargetY) - heading;
    }

    m_turnCtrl.setSetpoint(goalAngle);

    m_output = MathUtil.clamp(m_turnCtrl.calculate(heading), -1.0, 1.0);

    m_drive.drive(0.0, 0.0, m_output * ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
