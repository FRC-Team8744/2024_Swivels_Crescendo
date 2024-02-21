// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Intake extends SubsystemBase {
  public DigitalInput inputIR = new DigitalInput(0);
  public double intakeSpeed = 0.3;
  private CANSparkMax frontIntakeSparkMax = new CANSparkMax(MechanismConstants.kFrontIntakePort, MotorType.kBrushless);
  private CANSparkMax rearIntakeSparkMax = new CANSparkMax(MechanismConstants.kRearIntakePort, MotorType.kBrushless);
  /** Creates a new Intake. */
  public Intake() {

  }
  
  public void donutGrab(double speed) {
    frontIntakeSparkMax.set(speed * 3/2); //Ratio of wheel sizes
    rearIntakeSparkMax.set(speed);
  }

    public void donutRelease(double speed) {
    frontIntakeSparkMax.set(speed * 3/2); //Ratio of wheel sizes
    rearIntakeSparkMax.set(speed);
  }

  public void motorOff() {
    frontIntakeSparkMax.stopMotor();
    rearIntakeSparkMax.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
