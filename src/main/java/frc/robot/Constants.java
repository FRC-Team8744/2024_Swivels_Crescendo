// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public final class Constants {
  public static final int kDebugLevel = 0; // 0 = None, 1 = Errors, 2 = Info, 3 = Debug and USB data log
  
  public static final PowerDistribution PDH = new PowerDistribution(14, ModuleType.kRev); 

  public static final String controllerMode = "x";

  public static final int kMaxSpeedPercentAuto = 100;
  public static final int kMaxSpeedPercentTeleop = 100;
  public static final int kMaxAccelerationPercent = 100;
  public static final double kDriverSpeedLimitRot = 0.5; 
  public static final double kDriverSpeedLimitTran = 0.25; // sets how much the max speed is modified by when you press down on the left stick basicly make go slower the default is 1 btw 



  public static final class MechanismConstants {
    public static final int kFrontIntakePort = 15;
    public static final int kRearIntakePort = 16;
    public static final int kTopShooterPort = 17; // Runs forward
    public static final int kIndexShooterPort = 18;
    public static final int kBottomShooterPort = 19; // Runs backward
    public static final int kRightPivotShooterPort = 20;
    public static final int kLeftPivotShooterPort = 21; // Has absolute encoder
    public static final int kUndertakerIntakePort = 22;
    public static final int kLeftClimberPort = 23;
    public static final int kRightClimberPort = 24;
  }

  public static final class SwerveConstants {
    public static final double kMaxSpeedMetersPerSecond = (4.4 * kMaxSpeedPercentAuto) / 100;
    public static final double kMaxSpeedTeleop = (3.0 * kMaxSpeedPercentTeleop) / 100;

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

    // public static final boolean kFrontLeftTurningEncoderReversed = true;
    // public static final boolean kFrontRightTurningEncoderReversed = false;
    // public static final boolean kRearLeftTurningEncoderReversed = true;
    // public static final boolean kRearRightTurningEncoderReversed = true;

    // public static final boolean kFrontLeftDriveEncoderReversed = false;
    // public static final boolean kFrontRightDriveEncoderReversed = false;
    // public static final boolean kRearLeftDriveEncoderReversed = true;
    // public static final boolean kRearRightDriveEncoderReversed = true;

    public static final int kFrontLeftMagEncoderPort = 12;
    public static final int kFrontRightMagEncoderPort = 9;
    public static final int kRearLeftMagEncoderPort = 11;
    public static final int kRearRightMagEncoderPort = 10;

    // Only disable the steering angle optimizer when measuring the CANcoder offsets!
    public static final boolean DISABLE_ANGLE_OPTIMIZER = false;

    // Note: Zeroing the CanCoder in Tuner X doesn't seem to affect the reported absolute position.
    public static final double kFrontLeftMagEncoderOffsetDegrees_NoNo = 240.55;
    public static final double kFrontRightMagEncoderOffsetDegrees_NoNo = 317.94;
    public static final double kRearLeftMagEncoderOffsetDegrees_NoNo = 241.87;
    public static final double kRearRightMagEncoderOffsetDegrees_NoNo = 133.46;

    // public static final double kFrontLeftMagEncoderOffsetDegrees_NoNo = 0.876221 * 360;
    // public static final double kFrontRightMagEncoderOffsetDegrees_NoNo = 0.364990 * 360;
    // public static final double kRearLeftMagEncoderOffsetDegrees_NoNo = 0.670898 * 360;
    // public static final double kRearRightMagEncoderOffsetDegrees_NoNo = 0.666260 * 360;

    public static final double kFrontLeftMagEncoderOffsetDegrees_Swivels = 81.12;
    public static final double kFrontRightMagEncoderOffsetDegrees_Swivels = 133.77;
    public static final double kRearLeftMagEncoderOffsetDegrees_Swivels = 11.25;
    public static final double kRearRightMagEncoderOffsetDegrees_Swivels = 66.71;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(20.75);

    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20.75);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
          // This is the order all swerve module references need to be in!
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),  // Front Left Quadrant
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  // Front Right Quadrant
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),  // Rear Left Quadrant
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));  // Rear Right Quadrant

    public static final int kIMU_ID = 13;
    // public static final boolean kGyroReversed = false;

    public static int kSwerveFL_enum = 0;
    public static int kSwerveFR_enum = 1;
    public static int kSwerveRL_enum = 2;
    public static int kSwerveRR_enum = 3;
  }

  public static final class ConstantsOffboard {
    public static final int kMaximumSparkMaxRPM = 5700;
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double DRIVE_GEAR_RATIO = 6.75 / 1.0; // 6.75:1
    public static final double DRIVE_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    public static final double DRIVE_RPM_TO_METERS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;
    public static final double ANGLE_GEAR_RATIO = (150 / 7) / 1.0; // 150/7:1
    public static final double ANGLE_ROTATIONS_TO_RADIANS = (Math.PI * 2) / ANGLE_GEAR_RATIO;
    public static final double ANGLE_RPM_TO_RADIANS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;

    /** Speed ramp. */
    // public static final double OPEN_LOOP_RAMP = 0.25;
    // public static final double CLOSED_LOOP_RAMP = 0.0;

    /** Current limiting. */
    public static final int DRIVE_CURRENT_LIMIT = 40;
    public static final int ANGLE_CURRENT_LIMIT = 25;

    public static final boolean DRIVE_MOTOR_PROFILED_MODE = true;
    /** Angle motor PID values for speed/acceleration limited mode. */
    // Reference: https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
    public static final double DRIVE_KP_PROFILED = 0.01;
    public static final double DRIVE_KI_PROFILED = 0.0;
    public static final double DRIVE_KD_PROFILED = 0.0;
    public static final double DRIVE_KF_PROFILED = 0.23;
    public static final double DRIVE_MAX_VEL_PROFILED = kMaximumSparkMaxRPM;  // Maximum Velocity, RPM
    public static final double DRIVE_MAX_ACC_PROFILED = 20000;  // Maximum Acceleration, RPM^2
    public static final double DRIVE_MAX_ERR_PROFILED = 0.02;  // Error tolerance of PID controller, rotations

    /** Drive motor PID values. */
    public static final double DRIVE_KP = 0.1;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = 0.25;

    public static final boolean ANGLE_MOTOR_PROFILED_MODE = true;
    /** Angle motor PID values for speed/acceleration limited mode. */
    // Reference: https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
    public static final double ANGLE_KP_PROFILED = 0.00001;
    public static final double ANGLE_KI_PROFILED = 0.0;
    public static final double ANGLE_KD_PROFILED = 0.0;
    public static final double ANGLE_KF_PROFILED = 0.0003;
    public static final double ANGLE_MAX_VEL_PROFILED = kMaximumSparkMaxRPM;  // Maximum Velocity, RPM
    public static final double ANGLE_MAX_ACC_PROFILED = 20000;  // Maximum Acceleration, RPM^2
    public static final double ANGLE_MAX_ERR_PROFILED = 0.02;  // Error tolerance of PID controller, rotations

    /** Angle motor PID values. */
    public static final double ANGLE_KP = 1.5;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.1;
    public static final double ANGLE_KF = 0.0;
    
    /** Swerve constraints. */
    public static final double MAX_SPEED_IN_PERCENT = 100.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.0442 * MAX_SPEED_IN_PERCENT;
    public static final double MAX_ANGULAR_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND * 4/3;
    public static final double MAX_ANGULAR_DEGREES_PER_SECOND = Math.toDegrees(MAX_ANGULAR_RADIANS_PER_SECOND);

    /** Inversions. */
    public static final boolean DRIVE_MOTOR_INVERSION = true;
    public static final boolean ANGLE_MOTOR_INVERSION = true;
    // public static final boolean CANCODER_INVERSION = false;

    /** Idle modes. */
    // public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
    // public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kBrake;

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCodriverControllerPort = 1;
    public static final double kDeadband = 0.3;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = (4.4 * kMaxSpeedPercentAuto) / 100;
    public static final double kMaxAccelerationMetersPerSecondSquared = (3 * kMaxAccelerationPercent) / 100;
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

  public static final class TrajectoryConstants {
    public static final Translation2d blueWingNote1 = new Translation2d(Units.inchesToMeters(114), Units.inchesToMeters(161.5));
    public static final Translation2d blueWingNote2 = new Translation2d(Units.inchesToMeters(114), Units.inchesToMeters(218.5));
    public static final Translation2d blueWingNote3 = new Translation2d(Units.inchesToMeters(114), Units.inchesToMeters(275.5));

    public static final Translation2d redWingNote1 = new Translation2d(16.4592 - Units.inchesToMeters(114), Units.inchesToMeters(161.5));
    public static final Translation2d redWingNote2 = new Translation2d(16.4592 - Units.inchesToMeters(114), Units.inchesToMeters(218.5));
    public static final Translation2d redWingNote3 = new Translation2d(16.4592 - Units.inchesToMeters(114), Units.inchesToMeters(275.5));

    public static final Translation2d blueStart = new Translation2d(1.30, 5.56);
    public static final Translation2d redStart = new Translation2d(16.4592 - 1.30, 5.56);
  }
}
