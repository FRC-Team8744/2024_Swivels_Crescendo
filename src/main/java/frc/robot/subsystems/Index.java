// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Index extends SubsystemBase {
  public double indexSpeed = 1;
    public DigitalInput inputIR = new DigitalInput(0);
  private CANSparkMax indexSparkMax = new CANSparkMax(MechanismConstants.kIndexShooterPort, MotorType.kBrushless);
  
  public Index() {

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("Sensor", inputIR.get());
  }
}