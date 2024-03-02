// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Vision2;

public class auto_led extends Command {
  private final Vision2 m_vision;
  private final LEDS m_lightbarLeds;
  private final DriveSubsystem m_drive;
    PIDController m_turnCtrl = new PIDController(5, 0, 0);
  private static final double kTurnFF = 0.0;
  private double m_output;
  private double m_heading;
  // private double m_goalAngle = 0.0;
  private double tv;
  private int x; // x tracks angle.
  private double tx;
  private boolean Done;
  public int spi;
  private double goAngle;
  private static boolean ControlButtonB;
  private boolean NoTargetAtInit;

 private Debouncer m_debouncer = new Debouncer (0.1, Debouncer.DebounceType.kBoth );

  /** Creates a new auto_led. */
  public auto_led(Vision2 vision, LEDS light, DriveSubsystem drive) {
    m_vision = vision;
    addRequirements(m_vision);
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
    m_turnCtrl.setTolerance(2.0);
    // m_turnCtrl.setSetpoint(m_goalAngle);
    m_turnCtrl.reset();
    m_heading = m_drive.m_imu.getHeadingDegrees();

    NoTargetAtInit = m_vision.isSpeakerInView();
  
    // if (NoTargetAtInit) {
      tx = SmartDashboard.getNumber("tx",0);
      goAngle = (m_heading - tx);
    // } else {
      // goAngle = m_heading;
    // }
    
    m_turnCtrl.setSetpoint(goAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // ControlButtonB = !ControlButtonB;
    if (false) { //ControlButtonB){
      tv = SmartDashboard.getNumber("tv", 0);
      tx = SmartDashboard.getNumber("LimelightX", 0);
    } else  {
      tv = SmartDashboard.getBoolean("RT", false)?1:0;
    // tx = SmartDashboard.getNumber("tx", 0);
    // goAngle = (-tx * (36.0/30.0) + 17.0);
    // m_turnCtrl.setSetpoint(goAngle);
    }
    // x = (int) (-tx * (36.0/30.0) + 17.0);//LED conversion.
    if (tv == 1){
      m_lightbarLeds.allOff();
      m_lightbarLeds.setLed(x,1,255,1); //green
    } else {
      m_lightbarLeds.allOff(); // after it's done it will be red
      m_lightbarLeds.setLed(17, 255, 1, 1); // red
    }
    m_heading = m_drive.m_imu.getHeadingDegrees();

  SmartDashboard.putBoolean("NoTargetATInit", NoTargetAtInit);

        // if (tx >= 0){
        //   m_goalAngle = m_heading + tx;
        // }
        // if (tx <= 0){
        //   m_goalAngle = m_heading - tx;
        // }
        // m_goalAngle = m_heading + tx;
        // m_turnCtrl.setSetpoint(m_goalAngle);
        // goAngle = (-tx * (36.0/30.0) + 17.0);
        // m_turnCtrl.setSetpoint(goAngle);
    m_output = MathUtil.clamp(m_turnCtrl.calculate(m_heading) + kTurnFF, -1.0, 1.0);
    // Send PID output to drivebase
    // if (tx >= 0){
    //   m_drive.drive(0.0, 0.0, -m_output, false);
    // }
    // if (tx <= 0){
    //   m_drive.drive(0.0, 0.0, -m_output, false);
    // }
    m_drive.drive(0.0, 0.0, m_output, false);

    // Debug information
    SmartDashboard.putNumber("heading", m_heading);
    SmartDashboard.putData("PID", m_turnCtrl);
    SmartDashboard.putNumber("goangle", goAngle);
    // SmartDashboard.putNumber("PID setpoint", m_goalAngle);
    SmartDashboard.putNumber("PID output", m_output);
    SmartDashboard.putNumber("PID setpoint error", m_turnCtrl.getPositionError());
    SmartDashboard.putNumber("PID velocity error", m_turnCtrl.getVelocityError());
    SmartDashboard.putNumber("PID measurement", m_heading);
    // SmartDashboard.putBoolean("Done", m_turnCtrl.atSetpoint());
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
    if (NoTargetAtInit) return true;
    
    Done = false;
    SmartDashboard.putBoolean("Done", m_turnCtrl.atSetpoint());
    if (m_turnCtrl.atSetpoint()) Done = true;
    return m_turnCtrl.atSetpoint();
    // return Done;
  }
}
