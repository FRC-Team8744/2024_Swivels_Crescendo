// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Vision2;

public class TringsTest extends Command {
  private final LEDS m_lightbarLeds;
  private final DriveSubsystem m_drive;
  private final Vision2 m_vision2;
  PIDController m_turnCtrl = new PIDController(5, 0, 0);
  private static final double kTurnFF = 0.0;
  private double m_output;
  private double m_heading;
  private double tv;
  private int x; // x tracks angle.
  private double tx;
  private boolean Done;

  public TringsTest(LEDS light, DriveSubsystem drive, Vision2 vision2) {
    m_lightbarLeds = light;
    addRequirements(m_lightbarLeds);
    m_drive = drive;
    addRequirements(m_drive);
    m_vision2 = vision2;
    addRequirements(m_vision2);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double yDistance = m_vision2.getTargetYDistance().orElse(0.0);
    if (yDistance < 5.54)  m_vision2.m_goalAngle = (Math.toDegrees(Math.atan2(yDistance, m_vision2.getTargetDistance())));
    else m_vision2.m_goalAngle = -1 * Math.toDegrees(Math.atan2(Math.abs(yDistance - 5.54), m_vision2.getTargetDistance()));
    m_lightbarLeds.setLed(17,255,255,0);
    m_turnCtrl.enableContinuousInput(-180, 180);
    m_turnCtrl.setTolerance(3.0);
    m_turnCtrl.setSetpoint(m_vision2.m_goalAngle);
    m_turnCtrl.reset();

SmartDashboard.putData("PID", m_turnCtrl);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tv = SmartDashboard.getNumber("tv", 0);
    tx = SmartDashboard.getNumber("LimelightX", 0);
    x = (int) (-tx * (36.0/30.0) + 17.0);//LED conversion.
    if (tv == 1){
      m_lightbarLeds.allOff();
      m_lightbarLeds.setLed(x,1,255,1); //green
    } else {
      m_lightbarLeds.allOff(); // after it's done it will be red
      m_lightbarLeds.setLed(17, 255, 1, 1); // red
    }
        m_heading = m_drive.m_imu.getHeadingDegrees();
        // if (tx >= 0){
        //   m_goalAngle = m_heading + tx;
        // }
        // if (tx <= 0){
        //   m_goalAngle = m_heading - tx;
        // }
        // m_vision2.m_goalAngle = m_heading + tx;
        m_turnCtrl.setSetpoint(m_vision2.m_goalAngle);
    m_output = MathUtil.clamp(m_turnCtrl.calculate(m_heading) + kTurnFF, -1.0, 1.0);
    // Send PID output to drivebase
    // if (tx >= 0){
    //   m_drive.drive(0.0, 0.0, -m_output, false);
    // }
    // if (tx <= 0){
    //   m_drive.drive(0.0, 0.0, -m_output, false);
    // }
    m_drive.drive(0.0, 0.0, -m_output, false);

    // Debug information
    SmartDashboard.putNumber("PID setpoint", m_vision2.m_goalAngle);
    SmartDashboard.putNumber("PID output", m_output);
    SmartDashboard.putNumber("PID setpoint error", m_turnCtrl.getPositionError());
    SmartDashboard.putNumber("PID velocity error", m_turnCtrl.getVelocityError());
    SmartDashboard.putNumber("PID measurement", m_heading);
    // SmartDashboard.putNumber("Target Y distance", yDistance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   m_lightbarLeds.allOff();
   m_drive.drive(0.0, 0.0, 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Done = false;
    if (m_turnCtrl.atSetpoint()) Done = true;
    if (tv == 0) Done = true;

    return Done;
  }
}

