// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDS;

public class AprilTagID extends Command {
  private final LEDS m_lightbarLeds;
  private final DriveSubsystem m_drive;
    PIDController m_turnCtrl = new PIDController(5, 0, 0);
  private static final double kTurnFF = 0.0;
  private double m_output;
  private double m_heading;
  private double m_goalAngle = 0.0;
  private double tv;
  private int x; // x tracks angle.
  private double tx;
  private boolean Done;
  public int spi;
  private double m_AprilNumber;
   NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      NetworkTableEntry id = table.getEntry("tid");
  
  /** Creates a new auto_led. */
  public AprilTagID(LEDS light, DriveSubsystem drive) {
    m_lightbarLeds = light;
    addRequirements(m_lightbarLeds);
    m_drive = drive;
    addRequirements(m_drive);


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
m_lightbarLeds.setLed(17,255,255,0);
m_turnCtrl.enableContinuousInput(-180, 180);
m_turnCtrl.setTolerance(3.0);
m_turnCtrl.setSetpoint(m_goalAngle);
m_turnCtrl.reset();
m_AprilNumber = id.getDouble(0.0);


// SmartDashboard.putData("PID", m_turnCtrl);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_AprilNumber = id.getDouble(0.0);

        if (m_AprilNumber == 3){
          // SmartDashboard.putNumber("yes", 3);
        } else {
          if (m_AprilNumber == 4){
            // SmartDashboard.putNumber("yes",4);
          } else {
            // SmartDashboard.putNumber("no", -1);
          }
        }

        m_goalAngle = m_heading + tx;
        m_turnCtrl.setSetpoint(m_goalAngle);
    m_output = MathUtil.clamp(m_turnCtrl.calculate(m_heading) + kTurnFF, -1.0, 1.0);

    m_drive.drive(0.0, 0.0, -m_output, false);

    // Debug information
    // SmartDashboard.putNumber("PID setpoint", m_goalAngle);
    // SmartDashboard.putNumber("PID output", m_output);
    // SmartDashboard.putNumber("PID setpoint error", m_turnCtrl.getPositionError());
    // SmartDashboard.putNumber("PID velocity error", m_turnCtrl.getVelocityError());
    // SmartDashboard.putNumber("PID measurement", m_heading);
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
    if (m_turnCtrl.atSetpoint()) Done=true;
    if (tv == 0) Done=true;

    return Done;
  }
}
