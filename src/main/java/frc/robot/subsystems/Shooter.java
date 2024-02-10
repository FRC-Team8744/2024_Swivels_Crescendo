// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Shooter extends SubsystemBase {
  private CANSparkMax topShooterSparkMax = new CANSparkMax(MechanismConstants.kTopShooterPort, MotorType.kBrushless);
  private CANSparkMax bottomShooterSparkMax = new CANSparkMax(MechanismConstants.kBottomShooterPort, MotorType.kBrushless);
  private CANSparkMax indexSparkMax = new CANSparkMax(MechanismConstants.kIndexShooterPort, MotorType.kBrushless);
  private CANSparkMax leftPivotSparkMax = new CANSparkMax(MechanismConstants.kPLeftPivotShooterPort, MotorType.kBrushless); 
  private CANSparkMax rightPivotSparkMax = new CANSparkMax(MechanismConstants.kRightPivotShooterPort, MotorType.kBrushless); 
  
  public Shooter() {
    leftPivotSparkMax.setSmartCurrentLimit(10);
    rightPivotSparkMax.setSmartCurrentLimit(10);
    // rightPivotSparkMax.follow(leftPivotSparkMax);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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

  public void indexStop() {
    indexSparkMax.stopMotor();
  }

  public void testAngle(double speed) {
    leftPivotSparkMax.set(speed);
    rightPivotSparkMax.set(speed);
  }

  public void stopAngle() {
    leftPivotSparkMax.stopMotor();
    rightPivotSparkMax.stopMotor();
  }
}

// .32 maximum vert. .2 minimum vert.