// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

public class Multi_IMU extends SubsystemBase {
// TODO: CONSTANTS - move to Constants.java after debug
  public static final int DEBUG_NONE = 0;  // Debug levels: 0 = none, 1 = data to dashboard
  public static final int DEBUG_ALL = 1;  // Debug levels: 0 = none, 1 = data to dashboard
  public static final int DEBUG_IMU = DEBUG_ALL;  // Debug levels: 0 = none, 1 = data to dashboard

  // IMU types
  public static final int PIGEON1 = 0;  // https://store.ctr-electronics.com/gadgeteer-pigeon-imu/
  public static final int NAVX2_MICRO = 1;  // https://www.kauailabs.com/navx-micro/
  public static final int PIGEON2 = 2;  // https://store.ctr-electronics.com/pigeon-2/

  // IMU selected for data output
  public static final int IMU_SELECTED = PIGEON1;

  // PIGEON1
  // NOTES: Pigeon1 requires 5 seconds of zero robot motion after power up!
  public static final boolean PIGEON1_ENABLE = true;

  public static final int PIGEON1_kIMU_CAN_ID = 13;

  // NavX2 Micro
  // NOTES: NavX2 requires 5 seconds of zero robot motion after power up!
  // Specifications are equivalent to the Pigeon2
  // USB connection has a delay - try to connect with I2C
  public static final boolean NAVX2_MICRO_ENABLE = true;

  // PIGEON2 (Using Phoenix6 library)
  public static final boolean PIGEON2_ENABLE = true;

  public static final int PIGEON2_kIMU_CAN_ID = 19;  // TODO: what is this really?
// End CONSTANTS

  // The imu sensors
  private PigeonIMU m_imu_pigeon1;
  private AHRS m_imu_navX2_micro;
  private Pigeon2 m_imu_pigeon2;

  public Multi_IMU() {
    // Attempt to communicate with new sensor (may not exist)
    if (PIGEON1_ENABLE) m_imu_pigeon1 = new PigeonIMU(PIGEON1_kIMU_CAN_ID);

    if (NAVX2_MICRO_ENABLE) {
      try {
        m_imu_navX2_micro = new AHRS(SerialPort.Port.kUSB);
      } catch (RuntimeException ex) {
        DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
      }
    }

    if (PIGEON2_ENABLE) {
      m_imu_pigeon2 = new Pigeon2(PIGEON2_kIMU_CAN_ID, "rio");

      // Configure Pigeon2
      var toApply = new Pigeon2Configuration();  // User can change the configs if they want, or leave it empty for factory-default
      // Set configuration with "toApply." here
      /* For instance:
      toApply.MountPose.MountPoseYaw = 0;
      toApply.MountPose.MountPosePitch = 0;
      toApply.MountPose.MountPoseRoll = 90;
      */
      m_imu_pigeon2.getConfigurator().apply(toApply);

      // Speed up signals to an appropriate rate
      m_imu_pigeon2.getYaw().setUpdateFrequency(100);
      m_imu_pigeon2.getGravityVectorZ().setUpdateFrequency(100);
    }
  }

  @Override
  public void periodic() {
    if (PIGEON1_ENABLE && (DEBUG_IMU >= DEBUG_ALL)) {
      SmartDashboard.putNumber("Pigeon1 GyroZ", m_imu_pigeon1.getYaw());
      SmartDashboard.putNumber("Pigeon1 Absolute GyroZ", m_imu_pigeon1.getAbsoluteCompassHeading());
      SmartDashboard.putNumber("Pigeon1 Fused GyroZ", m_imu_pigeon1.getFusedHeading());
    }

    if (NAVX2_MICRO_ENABLE && (DEBUG_IMU >= DEBUG_ALL)) {
      SmartDashboard.putNumber("NavX GyroZ", m_imu_navX2_micro.getYaw());
      SmartDashboard.putNumber("NavX Fused GyroZ", m_imu_pigeon1.getFusedHeading());
    }

    if (PIGEON2_ENABLE && (DEBUG_IMU >= DEBUG_ALL)) {
      SmartDashboard.putNumber("Pigeon2 GyroZ", m_imu_pigeon2.getYaw().getValueAsDouble());
    }
  }

  /**
   * Zeros the heading of the robot.
   */
  public void zeroHeading() {
    if (PIGEON1_ENABLE) {
      m_imu_pigeon1.setYaw(0);
    }

    if (NAVX2_MICRO_ENABLE) {
      m_imu_navX2_micro.zeroYaw();
      m_imu_navX2_micro.reset();
    }

    if (PIGEON2_ENABLE) {
      m_imu_pigeon2.setYaw(0);
    }
  }

  /**
   * Returns the heading of the robot as Rotation2D
   *
   * @return the robot's heading as Rotation2D
   */
  public Rotation2d getHeading() {
    switch (IMU_SELECTED){
      case PIGEON1:
        return Rotation2d.fromDegrees(m_imu_pigeon1.getYaw());
      case NAVX2_MICRO:
        return Rotation2d.fromDegrees(m_imu_navX2_micro.getYaw());
      case PIGEON2:
        return Rotation2d.fromDegrees(m_imu_pigeon2.getYaw().getValueAsDouble());
      default:
        return Rotation2d.fromDegrees(0.0);
    }
  }

  /**
   * Returns human readable heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeadingDegrees() {
    switch (IMU_SELECTED){
      case PIGEON1:
        return m_imu_pigeon1.getYaw();
      case NAVX2_MICRO:
        return m_imu_navX2_micro.getYaw();
      case PIGEON2:
        return m_imu_pigeon2.getYaw().getValueAsDouble();
      default:
        return 9999999.9;
    }
  }

}
