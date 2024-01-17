// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private CANSparkMax rightFlywheelSparkMax = new CANSparkMax(Constants.kRightFlywheelShooterPort, MotorType.kBrushless);
  private CANSparkMax leftFlwheelSparkMax = new CANSparkMax(Constants.kLeftFlwheelShooterPort, MotorType.kBrushless);
  private CANSparkMax indexSparkMax = new CANSparkMax(Constants.kIndexShooterPort, MotorType.kBrushless);
  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
