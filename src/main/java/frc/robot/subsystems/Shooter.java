// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//KF 0.00005
//

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.MechanismConstants;

public class Shooter extends SubsystemBase {
  private static final double initialAngle = 9.0;
  private static final double shooterGearRatio = 15.0;
  private static final double minimumAngle = 9.0;
  private static final double maximumAngle = 70.0;
  public double shootingAngle = 60;
  public double shootingVelocity = 2500;
  // Wing 26/.65 for old 16ft
  public String shootingPreset = "Woofer";

  private CANSparkMax topShooterSparkMax = new CANSparkMax(MechanismConstants.kTopShooterPort, MotorType.kBrushless);
  private CANSparkMax bottomShooterSparkMax = new CANSparkMax(MechanismConstants.kBottomShooterPort, MotorType.kBrushless);
  private CANSparkMax leftPivotSparkMax = new CANSparkMax(MechanismConstants.kLeftPivotShooterPort, MotorType.kBrushless); 
  private CANSparkMax rightPivotSparkMax = new CANSparkMax(MechanismConstants.kRightPivotShooterPort, MotorType.kBrushless);
  
  private final RelativeEncoder topShooterEnc = topShooterSparkMax.getEncoder();
  private final RelativeEncoder bottomShooterEnc = bottomShooterSparkMax.getEncoder();
  private final RelativeEncoder leftPivotEnc = leftPivotSparkMax.getEncoder();
  private final RelativeEncoder rightPivotEnc = rightPivotSparkMax.getEncoder();

  private final SparkAbsoluteEncoder absoluteEncoder = leftPivotSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
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
    SmartDashboard.putNumber("Top shooter", topShooterEnc.getPosition());
    SmartDashboard.putNumber("Bottom shooter", bottomShooterEnc.getPosition());
    SmartDashboard.putNumber("Shooter degrees", leftPivotEnc.getPosition());
    SmartDashboard.putNumber("Right pivot", rightPivotEnc.getPosition());
    SmartDashboard.putNumber("Left pivot", leftPivotEnc.getPosition());
    SmartDashboard.putNumber("Abosulte encoder", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Flywheel top RPM", topShooterEnc.getVelocity());
    SmartDashboard.putNumber("Flywheel bottom RPM", bottomShooterEnc.getVelocity());
    SmartDashboard.putNumber("Current left", leftPivotSparkMax.getOutputCurrent());
    SmartDashboard.putNumber("Current right", rightPivotSparkMax.getOutputCurrent());
    SmartDashboard.putString("Shooting preset", shootingPreset);
    SmartDashboard.putNumber("Shooting RPM average", (bottomShooterEnc.getVelocity() + bottomShooterEnc.getVelocity()) / 2);
    SmartDashboard.putNumber("Shooting velocity", shootingVelocity);
    SmartDashboard.putNumber("Shooting angle", shootingAngle);
  }

  public void testShoot(double speed) {
    bottomShooterPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
    topShooterPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
    // topShooterSparkMax.set(speed);
    // bottomShooterSparkMax.set(-speed);
  }

  public void stopShooter() {
    topShooterSparkMax.stopMotor();
    bottomShooterSparkMax.stopMotor();
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
    if ((topShooterEnc.getVelocity()) >= (shootingVelocity * .95) 
    && (shootingAngle >= (absoluteEncoder.getPosition() * 0.95)) 
    && (shootingAngle <= (absoluteEncoder.getPosition() * 1.05))) 
    {
      return true;
    }
    else {
      return false;
    }
  }

  public void setShooterStuff(double angle, double velocity, String name) {
    // Velocity will be RPM
    shootingAngle = angle;
    shootingVelocity = velocity;
    shootingPreset = name;
  }

}

// .32 maximum vert. .2 minimum vert.