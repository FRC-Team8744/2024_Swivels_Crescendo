// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConstantsOffboard;

public class LockOnTarget {
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private Pose2d targetPose = aprilTagFieldLayout.getTagPose(7).get().toPose2d();
  private PIDController m_turnCtrl = new PIDController(0.02, 0.0003, 0.0015);
  private double goalAngle;
  private double heading;
  private double m_output;

  // Called when the command is initially scheduled.
  public void initialize(Pose2d estimatedPose2d) {
    m_turnCtrl.enableContinuousInput(-180, 180);
    m_turnCtrl.setTolerance(1.0);
    m_turnCtrl.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  public double execute(Pose2d estimatedPose2d) {
    heading = estimatedPose2d.getRotation().getDegrees();

    double distanceToTargetX = estimatedPose2d.getX() - targetPose.getX();
    double distanceToTargetY = estimatedPose2d.getY() - targetPose.getY();
    double distanceToTargetHyp = Math.abs(Math.sqrt(Math.pow(distanceToTargetX, 2) + Math.pow(distanceToTargetY, 2)));

    goalAngle = (Math.toDegrees(Math.atan(distanceToTargetY / distanceToTargetX)) - 6.0 + (2 * estimatedPose2d.getX() / 3));
    /* if (estimatedPose2d.getY() > targetPose.getY()) {
      goalAngle = Math.toDegrees(Math.atan(distanceToTargetX / distanceToTargetY)) + heading;
    }
    else {
      goalAngle = Math.toDegrees(Math.atan(distanceToTargetX / distanceToTargetY)) - heading;
    }

    if (goalAngle >= 0 && heading >= 0 ) {
      goalAngle = goalAngle - heading;
    }
    else if (goalAngle >= 0 && heading < 0) {
      goalAngle = goalAngle - heading;
    }
    else if (goalAngle < 0 && heading >= 0) {
      goalAngle = goalAngle - heading;
    }
    else {
      goalAngle = goalAngle - heading;
    }
    */

    // goalAngle = heading - goalAngle;

    // if (goalAngle > 180) {
    //   goalAngle = 360 - goalAngle;
    // }
    // else if (goalAngle < -180) {
    //   goalAngle = goalAngle + 360;
    // }

    // goalAngle = goalAngle + 90;

    SmartDashboard.putNumber("Goal Angle", goalAngle);
    // SmartDashboard.putNumber("Target Pose X", targetPose.getX());
    // SmartDashboard.putNumber("Target Pose Y", targetPose.getY());

    m_turnCtrl.setSetpoint(goalAngle);

    m_output = MathUtil.clamp(m_turnCtrl.calculate(heading), -1.0, 1.0);

    // m_output = MathUtil.clamp(heading, -1.0, 1.0);

    return m_output * ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND;
    // m_drive.drive(0.0, 0.0, m_output * ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND, true);
  }
}