// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Pivot extends SubsystemBase {
  private static final double initialAngle = 9.0;
  private static final double shooterGearRatio = 15.0;
  public static final double minimumAngle = 21.05;
  private static final double maximumAngle = 70.0;

  public double shootingAngle = 58;
  public double ampShootingAngle = 65.5;
  public double visionShootAngle = 26;

  private CANSparkMax leftPivotSparkMax = new CANSparkMax(MechanismConstants.kLeftPivotShooterPort, MotorType.kBrushless); 
  private CANSparkMax rightPivotSparkMax = new CANSparkMax(MechanismConstants.kRightPivotShooterPort, MotorType.kBrushless);
    
  private final RelativeEncoder leftPivotEnc = leftPivotSparkMax.getEncoder();

  public final SparkAbsoluteEncoder absoluteEncoder = leftPivotSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

  private final SparkPIDController leftPivotPID = leftPivotSparkMax.getPIDController();
  /** Creates a new Pivot. */
  public Pivot() {
    leftPivotSparkMax.restoreFactoryDefaults();
    rightPivotSparkMax.restoreFactoryDefaults();

    leftPivotSparkMax.setSmartCurrentLimit(40);
    rightPivotSparkMax.setSmartCurrentLimit(40);

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

    leftPivotPID.setFeedbackDevice(absoluteEncoder);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Abosulte encoder", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Shooting angle", shootingAngle);
    SmartDashboard.putNumber("Vision angle", visionShootAngle);
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
}
