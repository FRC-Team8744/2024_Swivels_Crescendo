// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Vector;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
  private double wheelCirc = 0.319185814;
  private double shooterVelocity;
  private Vector<Double> ShooterVector;

  // Called when the command is initially scheduled.
  public void initialize() {
    m_turnCtrl.enableContinuousInput(-180, 180);
    m_turnCtrl.setTolerance(1.0);
    m_turnCtrl.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  public double execute(Pose2d estimatedPose2d, Vector<Double> robotVector) {
    var alliance = DriverStation.getAlliance();
    if (alliance.get() == DriverStation.Alliance.Red) {
      targetPose = aprilTagFieldLayout.getTagPose(4).get().toPose2d();
    }
    else {
      targetPose = aprilTagFieldLayout.getTagPose(7).get().toPose2d();
    }
    heading = estimatedPose2d.getRotation().getDegrees();

    shooterVelocity = SmartDashboard.getNumber("Flywheel top RPM", 0.0);

    if (shooterVelocity <= 10) {
      shooterVelocity = 0;
    }

    SmartDashboard.putNumber("Robot Vector X", robotVector.get(0));
    SmartDashboard.putNumber("Robot Vector Y", robotVector.get(1));

    double distanceToTargetX = estimatedPose2d.getX() - targetPose.getX();
    double distanceToTargetY = estimatedPose2d.getY() - targetPose.getY();
    // double distanceToTargetHyp = Math.abs(Math.sqrt(Math.pow(distanceToTargetX, 2) + Math.pow(distanceToTargetY, 2)));

    if (alliance.get() == DriverStation.Alliance.Red) {
      goalAngle = (Math.toDegrees(Math.atan(distanceToTargetY / distanceToTargetX)) - 6 + (2 * (16.459 - estimatedPose2d.getX()) / 3)) - 180;
    }
    else {
      goalAngle = (Math.toDegrees(Math.atan(distanceToTargetY / distanceToTargetX)) - 6.0 + (2 * estimatedPose2d.getX() / 3));
    }
    SmartDashboard.putNumber("Goal Angle", goalAngle);
    // SmartDashboard.putNumber("Target Pose X", targetPose.getX());
    // SmartDashboard.putNumber("Target Pose Y", targetPose.getY());

    // ShooterVector = new Vector<>();

    // ShooterVector.add(shooterVelocity * wheelCirc * Math.toDegrees(Math.sin(Math.toRadians(goalAngle))));
    // ShooterVector.add(shooterVelocity * wheelCirc * Math.toDegrees(Math.cos(Math.toRadians(goalAngle))));

    // SmartDashboard.putNumber("Shooter Vector Sin", ShooterVector.get(0));
    // SmartDashboard.putNumber("Shooter Vector Cos", ShooterVector.get(1));

    // goalAngle += Math.toDegrees(Math.atan2(ShooterVector.get(0), ShooterVector.get(1)) - Math.atan2(robotVector.get(0), robotVector.get(1)));

    double v_sx = robotVector.get(0); // m/s, shooter velocity in x direction
    double v_sy = robotVector.get(1); // m/s, shooter velocity in y direction
    double n = SmartDashboard.getNumber("Flywheel top RPM", 0.0) * 60;   // rotations per second
    double C = 0.319185814;    // meters, circumference of the wheel
    double theta_s = goalAngle; // degrees, current facing angle of the shooter

    // Calculate projectile speed
    double v_p = n * C;

    // Convert theta_s to radians for calculation
    double theta_s_rad = Math.toRadians(theta_s);

    // Calculate projectile's initial velocity components
    double v_px = v_p * Math.cos(theta_s_rad);
    double v_py = v_p * Math.sin(theta_s_rad);

    // Calculate effective projectile velocity components
    double v_px_eff = v_px + v_sx;
    double v_py_eff = v_py + v_sy;

    // Calculate the effective shooting angle
    double theta_eff = Math.toDegrees(Math.atan2(v_py_eff, v_px_eff));

    SmartDashboard.putNumber("theta eff", theta_eff);
    SmartDashboard.putNumber("v_py_eff", v_py_eff);
    SmartDashboard.putNumber("v_px_eff", v_px_eff);

    if (n >= 100) {    
      m_turnCtrl.setSetpoint(theta_eff);
    }
    else {
      m_turnCtrl.setSetpoint(goalAngle);
    }

    m_output = MathUtil.clamp(m_turnCtrl.calculate(heading), -1.0, 1.0);

    // m_output = MathUtil.clamp(heading, -1.0, 1.0);

    return m_output * ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND;
    // m_drive.drive(0.0, 0.0, m_output * ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND, true);
  }
}