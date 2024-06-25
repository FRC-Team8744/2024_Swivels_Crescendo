// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision2;

public class auto_led extends Command {
  private final Vision2 m_vision;
  private final LEDS m_leds;
  private final DriveSubsystem m_drive;
  private final Pivot m_pivot;
  private final Shooter m_shooter;
  PIDController m_turnCtrl = new PIDController(0.02, 0, 0);
  private double m_output;
  private double m_heading;
  private int x; // x tracks angle.
  private double tx;
  private boolean Done;
  public int spi;
  private double goAngle;
  private boolean NoTargetAtInit;

 private Debouncer m_debouncer = new Debouncer (0.1, Debouncer.DebounceType.kBoth);

 private double angleOffset;

  /** Creates a new auto_led. */
  public auto_led(Vision2 vision, DriveSubsystem drive, LEDS leds, Shooter sh, Pivot pi) {
    m_vision = vision;
    addRequirements(m_vision);
    m_drive = drive;
    addRequirements(m_drive);
    m_leds = leds;
    addRequirements(m_leds);
    m_pivot = pi;
    // addRequirements(m_pivot);
    m_shooter = sh;
    addRequirements(m_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_leds.ledOn(128, 0, 0);
    m_turnCtrl.enableContinuousInput(-180, 180);
    m_turnCtrl.setTolerance(1.0);
    m_turnCtrl.reset();
    m_heading = m_drive.m_imu.getHeadingDegrees();

    NoTargetAtInit = m_vision.isSpeakerInView();
    PhotonTrackedTarget target = m_vision.getTarget();
    if (NoTargetAtInit && target != null) {
      if (m_vision.getTargetDistance() > 8.23) {m_pivot.visionShootAngle = Math.toDegrees(Math.atan(m_vision.getTargetVertAngle() / Math.abs(m_vision.getTargetDistance() - 16.459))) + Math.abs(m_vision.getTargetDistance() - 16.459) * 2/3 -1; }

      else {m_pivot.visionShootAngle = Math.toDegrees(Math.atan(m_vision.getTargetVertAngle() / m_vision.getTargetDistance())) + (m_vision.getTargetDistance() * 2/3 -1);}

      angleOffset = target.getYaw() -6.0 + (2 * m_vision.getTargetDistance() / 3);
      tx = SmartDashboard.getNumber("tx", 0);
      goAngle = m_heading - angleOffset;
      m_turnCtrl.setSetpoint(goAngle);
    } else {
      m_pivot.visionShootAngle = 32;
      m_leds.ledOn(255, 0, 0);
      m_turnCtrl.setSetpoint(m_heading);
    }
    m_shooter.testShoot(m_shooter.visionShootVelocity);
    // m_pivot.testAngle(m_pivot.visionShootAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Shooter Angle", m_pivot.visionShootAngle);

    m_heading = m_drive.m_imu.getHeadingDegrees();

    m_output = MathUtil.clamp(m_turnCtrl.calculate(m_heading), -1.0, 1.0);

    m_drive.drive(0.0, 0.0, m_output * ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND, false);

    // Debug information
    SmartDashboard.putNumber("heading", m_heading);
    SmartDashboard.putData("PID", m_turnCtrl);
    SmartDashboard.putNumber("goangle", goAngle);
    SmartDashboard.putNumber("PID output", m_output);
    SmartDashboard.putNumber("PID setpoint error", m_turnCtrl.getPositionError());
    SmartDashboard.putNumber("PID velocity error", m_turnCtrl.getVelocityError());
    SmartDashboard.putNumber("PID measurement", m_heading);
    SmartDashboard.putBoolean("Done", m_turnCtrl.atSetpoint());
    } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //  m_lightbarLeds.allOff();
   m_drive.drive(0.0, 0.0, 0.0, false);
   m_shooter.stopShooter();
  //  m_pivot.stopAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    Done = false;
    SmartDashboard.putBoolean("Done", m_turnCtrl.atSetpoint());
    if (m_turnCtrl.atSetpoint()) m_leds.ledOn(0, 128, 0);
    return m_turnCtrl.atSetpoint();
    // return Done;
  }
}
