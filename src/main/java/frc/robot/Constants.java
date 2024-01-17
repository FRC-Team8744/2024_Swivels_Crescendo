// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // TODO: Update motor port numbers
  public static final int kFrontIntakePort = 30;
  public static final int kRearIntakePort = 31;
  public static final int kRightFlywheelShooterPort = 32;
  public static final int kLeftFlwheelShooterPort = 33;
  public static final int kIndexShooterPort = 34;
  public static final int kRightClimberPort = 35;
  public static final int kLeftClimberPort = 36;

  public static final class SwerveConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;

    // The drive classes use the NWU axes convention (North-West-Up as external reference in the world frame).
    // The positive X axis points ahead, the positive Y axis points left, and the positive Z axis points up.
    // We use NWU here because the rest of the library, and math in general, use NWU axes convention.
    // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#axis-conventions
    public static final int kFrontLeftDriveMotorPort = 5;
    public static final int kFrontRightDriveMotorPort = 1;
    public static final int kRearLeftDriveMotorPort = 8;
    public static final int kRearRightDriveMotorPort = 4;

    public static final int kFrontLeftTurningMotorPort = 6;
    public static final int kFrontRightTurningMotorPort = 2;
    public static final int kRearLeftTurningMotorPort = 7;
    public static final int kRearRightTurningMotorPort = 3;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kRearRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kRearRightDriveEncoderReversed = true;

    public static final int kFrontLeftMagEncoderPort = 12;
    public static final int kFrontRightMagEncoderPort = 9;
    public static final int kRearLeftMagEncoderPort = 11;
    public static final int kRearRightMagEncoderPort = 10;

    public static final double kFrontLeftMagEncoderOffsetDegrees = 81.12;
    public static final double kFrontRightMagEncoderOffsetDegrees = 133.77;
    public static final double kRearLeftMagEncoderOffsetDegrees = 11.25;
    public static final double kRearRightMagEncoderOffsetDegrees = 66.71;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(20.472);

    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20.472);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
          // This is the order all swerve module references need to be in!
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),  // Front Left Quadrant
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  // Front Right Quadrant
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),  // Rear Left Quadrant
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));  // Rear Right Quadrant

    public static final int kIMU_ID = 13;
    public static final boolean kGyroReversed = false;

    public static int kSwerveFL_enum = 0;
    public static int kSwerveFR_enum = 1;
    public static int kSwerveRL_enum = 2;
    public static int kSwerveRR_enum = 3;
  }

  public static final class ConstantsOffboard {
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double DRIVE_GEAR_RATIO = 6.75 / 1.0; // 6.75:1
    public static final double DRIVE_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    public static final double DRIVE_RPM_TO_METERS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;
    public static final double ANGLE_GEAR_RATIO = (150 / 7) / 1.0; // 150/7:1
    public static final double ANGLE_ROTATIONS_TO_RADIANS = (Math.PI * 2) / ANGLE_GEAR_RATIO;
    public static final double ANGLE_RPM_TO_RADIANS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;

    /** Speed ramp. */
    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /** Current limiting. */
    public static final int DRIVE_CURRENT_LIMIT = 40;
    public static final int ANGLE_CURRENT_LIMIT = 20;

    /** Drive motor PID values. */
    public static final double DRIVE_KP = 0.1;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = 0.0;

    /** Drive motor characterization. */
    public static final double DRIVE_KS = 0.11937;
    public static final double DRIVE_KV = 2.6335;
    public static final double DRIVE_KA = 0.46034;

    /** Angle motor PID values. */
    public static final double ANGLE_KP = 1.5;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.1;
    public static final double ANGLE_KF = 0.0;
    
    /** Swerve constraints. */
    public static final double MAX_SPEED_IN_PERCENT = 100.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.0442 * MAX_SPEED_IN_PERCENT;
    public static final double MAX_ANGULAR_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND * 4/3;

    /** Inversions. */
    public static final boolean DRIVE_MOTOR_INVERSION = true;
    public static final boolean ANGLE_MOTOR_INVERSION = true;
    public static final boolean CANCODER_INVERSION = false;

    /** Idle modes. */
    // public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
    // public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kBrake;

    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kEncoderCPR = 1024;


    public static final double kWheelDiameterMeters = 0.15;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = 1;
  }

  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 0;
    public static final int kRearLeftDriveMotorPort = 2;
    public static final int kFrontRightDriveMotorPort = 4;
    public static final int kRearRightDriveMotorPort = 6;

    public static final int kFrontLeftTurningMotorPort = 1;
    public static final int kRearLeftTurningMotorPort = 3;
    public static final int kFrontRightTurningMotorPort = 5;
    public static final int kRearRightTurningMotorPort = 7;

    public static final int[] kFrontLeftTurningEncoderPorts = new int[] {0, 1};
    public static final int[] kRearLeftTurningEncoderPorts = new int[] {2, 3};
    public static final int[] kFrontRightTurningEncoderPorts = new int[] {4, 5};
    public static final int[] kRearRightTurningEncoderPorts = new int[] {6, 7};

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = true;

    public static final int[] kFrontLeftDriveEncoderPorts = new int[] {8, 9};
    public static final int[] kRearLeftDriveEncoderPorts = new int[] {10, 11};
    public static final int[] kFrontRightDriveEncoderPorts = new int[] {12, 13};
    public static final int[] kRearRightDriveEncoderPorts = new int[] {14, 15};

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = true;

    public static final double kTrackWidth = 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.7;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 3;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = 1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
