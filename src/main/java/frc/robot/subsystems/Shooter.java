// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Shooter extends SubsystemBase {
  private static final double initialAngle = 9.0;
  private static final double shooterGearRatio = 15.0;
  private static final double minimumAngle = 9.0;
  private static final double maximumAngle = 60.0;
  public double shootingAngle = 45;
  public double shootingVelocity = .5;

  private CANSparkMax topShooterSparkMax = new CANSparkMax(MechanismConstants.kTopShooterPort, MotorType.kBrushless);
  private CANSparkMax bottomShooterSparkMax = new CANSparkMax(MechanismConstants.kBottomShooterPort, MotorType.kBrushless);
  private CANSparkMax indexSparkMax = new CANSparkMax(MechanismConstants.kIndexShooterPort, MotorType.kBrushless);
  private CANSparkMax leftPivotSparkMax = new CANSparkMax(MechanismConstants.kPLeftPivotShooterPort, MotorType.kBrushless); 
  private CANSparkMax rightPivotSparkMax = new CANSparkMax(MechanismConstants.kRightPivotShooterPort, MotorType.kBrushless); 
  
  private final RelativeEncoder topShooterEnc = topShooterSparkMax.getEncoder();
  private final RelativeEncoder bottomShooterEnc = bottomShooterSparkMax.getEncoder();
  private final RelativeEncoder leftPivotEnc = leftPivotSparkMax.getEncoder();
  private final RelativeEncoder rightPivotEnc = rightPivotSparkMax.getEncoder();
  // private final RelativeEncoder absoluteShooter = rightPivotSparkMax.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

  private final SparkPIDController topShooterPID = topShooterSparkMax.getPIDController();
  private final SparkPIDController bottomShooterPID = bottomShooterSparkMax.getPIDController();
  private final SparkPIDController leftPivotPID = leftPivotSparkMax.getPIDController();
  private final SparkPIDController rightPivotPID = rightPivotSparkMax.getPIDController();

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
    // rightPivotSparkMax.burnFlash();
    // leftPivotSparkMax.burnFlash();

    // absoluteShooter.setPosition(0);

    leftPivotPID.setP(0.05);
    leftPivotPID.setI(0);
    leftPivotPID.setD(0);
    leftPivotPID.setFF(0);

    leftPivotEnc.setPositionConversionFactor(360 / shooterGearRatio);
    leftPivotEnc.setPosition(initialAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Top shooter", topShooterEnc.getPosition());
    SmartDashboard.putNumber("Bottom shooter", bottomShooterEnc.getPosition());
    SmartDashboard.putNumber("Shooter degrees", leftPivotEnc.getPosition());
    SmartDashboard.putNumber("Right pivot", rightPivotEnc.getPosition());
    // SmartDashboard.putNumber("Abosulte encoder", absoluteShooter.getPosition());
    SmartDashboard.putNumber("Flywheel top RPM", topShooterEnc.getVelocity());
    SmartDashboard.putNumber("Flywheel bottom RPM", bottomShooterEnc.getVelocity());
    SmartDashboard.putNumber("Current left", leftPivotSparkMax.getOutputCurrent());
    SmartDashboard.putNumber("Current right", rightPivotSparkMax.getOutputCurrent());
  }

  public void testShoot(double speed) {
    topShooterSparkMax.set(speed);
    bottomShooterSparkMax.set(-speed);
  }

  public void stopShooter() {
    topShooterSparkMax.stopMotor();
    bottomShooterSparkMax.stopMotor();
  }

  public void indexRun(double speed) {
    indexSparkMax.set(speed);
  }

  public void indexOut(double speed) {
    indexSparkMax.set(-speed);
  }

  public void indexStop() {
    indexSparkMax.stopMotor();
  }

  public void testAngle(double angle) {
    if (angle < minimumAngle) angle = minimumAngle;
    if (angle > maximumAngle) angle = maximumAngle;
    leftPivotPID.setReference(angle, CANSparkMax.ControlType.kPosition);
    // leftPivotSparkMax.set(speed);
    // rightPivotSparkMax.set(speed);
  }

  public void stopAngle() {
    leftPivotSparkMax.stopMotor();
    rightPivotSparkMax.stopMotor();
  }

  public boolean atSpeed() {
    if (topShooterEnc.getVelocity() >= 2500.0) {
      return true;
    }
    else {
      return false;
    }
  }

  public void setShooterStuff(double angle, double velocity) {
    shootingAngle = angle;
    shootingVelocity = velocity;
  }

}

// .32 maximum vert. .2 minimum vert.