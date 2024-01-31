// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  public DigitalInput inputIR = new DigitalInput(1);
  // private CANSparkMax frontIntakeSparkMax = new CANSparkMax(Constants.kFrontIntakePort, MotorType.kBrushless);
  // private CANSparkMax rearIntakeSparkMax = new CANSparkMax(Constants.kFrontIntakePort, MotorType.kBrushless);
  /** Creates a new Intake. */
  public Intake() {

  }
  public void donutGrab() {
  //   frontIntakeSparkMax.set(.8);
  //   rearIntakeSparkMax.set(.8);
  }
  public void motorOff() {
  //   frontIntakeSparkMax.stopMotor();
  //   rearIntakeSparkMax.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
