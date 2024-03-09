// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Intake extends SubsystemBase {
  public double intakeSpeed = 0.7;
  private CANSparkMax frontIntakeSparkMax = new CANSparkMax(MechanismConstants.kFrontIntakePort, MotorType.kBrushless);
  private CANSparkMax rearIntakeSparkMax = new CANSparkMax(MechanismConstants.kRearIntakePort, MotorType.kBrushless);
  private CANSparkMax undertakerSparkMax = new CANSparkMax(MechanismConstants.kUndertakerIntakePort, MotorType.kBrushless);
  /** Creates a new Intake. */
  public Intake() {

  }
  
  public void donutGrab(double speed) {
    frontIntakeSparkMax.set(-speed); 
    rearIntakeSparkMax.set(-speed * 2/3); //Ratio of wheel sizes
    undertakerSparkMax.set(speed);
  }
 
    public void donutRelease(double speed) {
    frontIntakeSparkMax.set(speed); 
    rearIntakeSparkMax.set(speed * 2/3); //Ratio of wheel sizes
    undertakerSparkMax.set(-speed);
  }

  public void motorOff() {
    frontIntakeSparkMax.stopMotor();
    rearIntakeSparkMax.stopMotor();
    undertakerSparkMax.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}