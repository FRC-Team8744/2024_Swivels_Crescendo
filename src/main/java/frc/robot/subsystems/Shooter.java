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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Shooter extends SubsystemBase {
  private static final double initialAngle = 9.0;
  private static final double shooterGearRatio = 15.0;
  private static final double minimumAngle = 9.0;
  private static final double maximumAngle = 70.0;
  public double shootingAngle = 58;
  public double shootingVelocity = 2430;
  public double ampShootingAngle = 65.5;
  public double ampShootingVelocity = 2150;
  public double visionShootAngle = 26;
  public double visionShootVelocity = 3780;
  public double ampTopShootingVelocity = ampShootingVelocity / 6;
  public String shootingPreset = "Woofer";

  private CANSparkMax topShooterSparkMax = new CANSparkMax(MechanismConstants.kTopShooterPort, MotorType.kBrushless);
  private CANSparkMax bottomShooterSparkMax = new CANSparkMax(MechanismConstants.kBottomShooterPort, MotorType.kBrushless);
  private CANSparkMax leftPivotSparkMax = new CANSparkMax(MechanismConstants.kLeftPivotShooterPort, MotorType.kBrushless); 
  private CANSparkMax rightPivotSparkMax = new CANSparkMax(MechanismConstants.kRightPivotShooterPort, MotorType.kBrushless);
  
  private final RelativeEncoder topShooterEnc = topShooterSparkMax.getEncoder();
  private final RelativeEncoder bottomShooterEnc = bottomShooterSparkMax.getEncoder();
  private final RelativeEncoder leftPivotEnc = leftPivotSparkMax.getEncoder();

  private final SparkAbsoluteEncoder absoluteEncoder = leftPivotSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

  private final SparkPIDController topShooterPID = topShooterSparkMax.getPIDController();
  private final SparkPIDController bottomShooterPID = bottomShooterSparkMax.getPIDController();
  private final SparkPIDController leftPivotPID = leftPivotSparkMax.getPIDController();

  public Shooter() {
    leftPivotSparkMax.restoreFactoryDefaults();
    rightPivotSparkMax.restoreFactoryDefaults();

    leftPivotSparkMax.setSmartCurrentLimit(40);
    rightPivotSparkMax.setSmartCurrentLimit(40);
    topShooterSparkMax.setSmartCurrentLimit(40);
    bottomShooterSparkMax.setSmartCurrentLimit(40);

    rightPivotSparkMax.follow(leftPivotSparkMax, true);

    leftPivotSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightPivotSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);

    leftPivotPID.setP(0.02);
    leftPivotPID.setI(0);
    leftPivotPID.setD(0);
    leftPivotPID.setFF(0.0015);

    leftPivotEnc.setPositionConversionFactor(360 / shooterGearRatio);
    absoluteEncoder.setPositionConversionFactor(360);
    leftPivotEnc.setPosition(initialAngle);

    bottomShooterSparkMax.setInverted(true);

    bottomShooterPID.setP(0.0001);
    bottomShooterPID.setI(0);
    bottomShooterPID.setD(0.001);
    bottomShooterPID.setFF(0.0002);

    topShooterPID.setP(0.0001);
    topShooterPID.setI(0);
    topShooterPID.setD(0.001);
    topShooterPID.setFF(0.0002);

    leftPivotPID.setFeedbackDevice(absoluteEncoder);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Abosulte encoder", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Flywheel top RPM", topShooterEnc.getVelocity());
    SmartDashboard.putNumber("Flywheel bottom RPM", bottomShooterEnc.getVelocity());
    SmartDashboard.putString("Shooting preset", shootingPreset);
    SmartDashboard.putNumber("Shooting velocity", shootingVelocity);
    SmartDashboard.putNumber("Shooting angle", shootingAngle);
    SmartDashboard.putNumber("Vision velocity", visionShootVelocity);
    SmartDashboard.putNumber("Vision angle", visionShootAngle);
  }

  public void testShoot(double speed) {
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

  public void testAngle(double angle) {
    if (angle < minimumAngle) angle = minimumAngle;
    if (angle > maximumAngle) angle = maximumAngle;
    leftPivotPID.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public void testAngleAmp(double angle) {
    angle -= 3.5;
    if (angle < minimumAngle) angle = minimumAngle;
    if (angle > maximumAngle) angle = maximumAngle;
    leftPivotPID.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public void stopAngle() {
    leftPivotSparkMax.stopMotor();
    rightPivotSparkMax.stopMotor();
  }

  public boolean atSpeed() {
    if ((topShooterEnc.getVelocity()) >= (shootingVelocity * .95)
    && (shootingAngle >= (absoluteEncoder.getPosition() * 0.9))
    && (shootingAngle <= (absoluteEncoder.getPosition() * 1.1))) 
    {
      return true;
    }
    else {
      return false;
    }
  }

  public boolean visionAtSpeed() {
    if ((topShooterEnc.getVelocity()) >= (visionShootVelocity * .95)
    && (visionShootAngle >= (absoluteEncoder.getPosition() * 0.9))
    && (visionShootAngle <= (absoluteEncoder.getPosition() * 1.1))) 
    {
      return true;
    }
    else {
      return false;
    }
  }

  public boolean ampAtSpeed() {
    if ((bottomShooterEnc.getVelocity()) >= (ampShootingVelocity * 0.9)
    && (ampShootingAngle >= ((absoluteEncoder.getPosition()) * 0.95))
    && (ampShootingAngle <= ((absoluteEncoder.getPosition()) * 1.05)))
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
    shootingAngle = angle;
    shootingVelocity = velocity;
    shootingPreset = name;
  }

}