// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.units.Power;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Shooter extends SubsystemBase {
  public double shootingVelocity = 2430;
  public double ampShootingVelocity = 2150;
  public double visionShootVelocity = 2500;
  public double visionShootVelocityLimit = 3750;
  public double ampTopShootingVelocity = ampShootingVelocity / 6;
  public String shootingPreset = "Woofer";

  public PowerDistribution PDH = new PowerDistribution(14, ModuleType.kRev);

  private CANSparkMax topShooterSparkMax = new CANSparkMax(MechanismConstants.kTopShooterPort, MotorType.kBrushless);
  private CANSparkMax bottomShooterSparkMax = new CANSparkMax(MechanismConstants.kBottomShooterPort, MotorType.kBrushless);
  
  private final RelativeEncoder topShooterEnc = topShooterSparkMax.getEncoder();
  private final RelativeEncoder bottomShooterEnc = bottomShooterSparkMax.getEncoder();

  private final SparkPIDController topShooterPID = topShooterSparkMax.getPIDController();
  private final SparkPIDController bottomShooterPID = bottomShooterSparkMax.getPIDController();

  public final Pivot m_pivot = new Pivot();

  public Shooter() {
    topShooterSparkMax.setSmartCurrentLimit(40);
    bottomShooterSparkMax.setSmartCurrentLimit(40);

    bottomShooterSparkMax.setInverted(true);

    bottomShooterPID.setP(0.0001);
    bottomShooterPID.setI(0);
    bottomShooterPID.setD(0.001);
    bottomShooterPID.setFF(0.0002);

    topShooterPID.setP(0.0001);
    topShooterPID.setI(0);
    topShooterPID.setD(0.001);
    topShooterPID.setFF(0.0002);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Flywheel top RPM", topShooterEnc.getVelocity());
    SmartDashboard.putNumber("Flywheel bottom RPM", bottomShooterEnc.getVelocity());
    // SmartDashboard.putString("Shooting preset", shootingPreset);
    // SmartDashboard.putNumber("Shooting velocity", shootingVelocity);
    SmartDashboard.putNumber("Vision velocity", visionShootVelocity);
  }

  public void testShoot(double speed) {
    if (speed >= visionShootVelocityLimit)  {
      speed = visionShootVelocityLimit;
    }
    bottomShooterPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
    topShooterPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }

  public void ampShoot(double topSpeed, double bottomSpeed) {
    topShooterPID.setReference(topSpeed, CANSparkMax.ControlType.kVelocity);
    bottomShooterPID.setReference(bottomSpeed, CANSparkMax.ControlType.kVelocity);
  }

  public void stopShooter() {
    topShooterSparkMax.stopMotor();
    bottomShooterSparkMax.stopMotor();
  }

  public boolean atSpeed() {
    if ((topShooterEnc.getVelocity()) >= (shootingVelocity * .95)
    && (m_pivot.shootingAngle >= (m_pivot.absoluteEncoder.getPosition() * 0.9))
    && (m_pivot.shootingAngle <= (m_pivot.absoluteEncoder.getPosition() * 1.1))) 
    {
      return true;
    }
    else {
      return false;
    }
  }

  public boolean visionAtSpeed() {
    if ((topShooterEnc.getVelocity()) >= (visionShootVelocity * .95)
    && (m_pivot.visionShootAngle >= (m_pivot.absoluteEncoder.getPosition() * 0.9))
    && (m_pivot.visionShootAngle <= (m_pivot.absoluteEncoder.getPosition() * 1.1))) 
    {
      return true;
    }
    else {
      return false;
    }
  }

  public boolean ampAtSpeed() {
    if ((bottomShooterEnc.getVelocity()) >= (ampShootingVelocity * 0.9)
    && (m_pivot.ampShootingAngle >= ((m_pivot.absoluteEncoder.getPosition()) * 0.95))
    && (m_pivot.ampShootingAngle <= ((m_pivot.absoluteEncoder.getPosition()) * 1.05)))
    {
      return true;
    }
    else {
      return false;
    }
  }

  /**
   * Configures the preset of the shooter
   *
   * @param angle sets the preset of the shooter's angle.
   * @param velocity sets the preset of the shooter's velocity
   * @param name sets the name of the shooter preset
   */

  public void setShooterStuff(double angle, double velocity, String name) {
    // Velocity will be RPM
    m_pivot.shootingAngle = angle;
    shootingVelocity = velocity;
    shootingPreset = name;
  }

}