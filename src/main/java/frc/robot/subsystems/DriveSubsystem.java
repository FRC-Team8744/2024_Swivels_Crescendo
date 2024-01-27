// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

// import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  StructPublisher<Pose2d> pose_publisher = NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose2d.struct).publish();
  StructArrayPublisher<SwerveModuleState> swerve_publisher = NetworkTableInstance.getDefault().getStructArrayTopic("Swerve States", SwerveModuleState.struct).publish();


  DigitalInput input = new DigitalInput(0);
  DigitalInput inputIR = new DigitalInput(1);

  double offset_FL = 0;
  double offset_RL = 0;
  double offset_FR = 0;
  double offset_RR = 0;
  
  // Robot swerve modules
  private final SwerveModuleOffboard m_frontLeft;
  //  =
  //   new SwerveModuleOffboard(
  //     SwerveConstants.kFrontLeftDriveMotorPort,
  //     SwerveConstants.kFrontLeftTurningMotorPort,
  //     SwerveConstants.kFrontLeftMagEncoderPort,
  //     SwerveConstants.kFrontLeftMagEncoderOffsetDegrees);

  private final SwerveModuleOffboard m_rearLeft;
  //  =
  //   new SwerveModuleOffboard(
  //     SwerveConstants.kRearLeftDriveMotorPort,
  //     SwerveConstants.kRearLeftTurningMotorPort,
  //     SwerveConstants.kRearLeftMagEncoderPort,
  //     SwerveConstants.kRearLeftMagEncoderOffsetDegrees);

  private final SwerveModuleOffboard m_frontRight;
  //  =
  //   new SwerveModuleOffboard(
  //     SwerveConstants.kFrontRightDriveMotorPort,
  //     SwerveConstants.kFrontRightTurningMotorPort,
  //     SwerveConstants.kFrontRightMagEncoderPort,
  //     SwerveConstants.kFrontRightMagEncoderOffsetDegrees);

  private final SwerveModuleOffboard m_rearRight;
  //  =
  //   new SwerveModuleOffboard(
  //     SwerveConstants.kRearRightDriveMotorPort,
  //     SwerveConstants.kRearRightTurningMotorPort,
  //     SwerveConstants.kRearRightMagEncoderPort,
  //     SwerveConstants.kRearRightMagEncoderOffsetDegrees);

  // The imu sensor
  private final PigeonIMU m_imu = new PigeonIMU(SwerveConstants.kIMU_ID);
  private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;
  //  =
  //     new SwerveDriveOdometry(
  //         SwerveConstants.kDriveKinematics,
  //         Rotation2d.fromDegrees(m_imu.getYaw()),
  //         new SwerveModulePosition[] {
  //           m_frontLeft.getPosition(),
  //           m_frontRight.getPosition(),
  //           m_rearLeft.getPosition(),
  //           m_rearRight.getPosition()
  //         });

  // Create Field2d for robot and trajectory visualizations.
  public Field2d m_field;
  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    switch(Preferences.getString("RobotName", "NoDefault")) {
      case "Swivels":
        offset_FL = SwerveConstants.kFrontLeftMagEncoderOffsetDegrees_Swivels;
        offset_RL = SwerveConstants.kRearLeftMagEncoderOffsetDegrees_Swivels;
        offset_FR = SwerveConstants.kFrontRightMagEncoderOffsetDegrees_Swivels;
        offset_RR = SwerveConstants.kRearRightMagEncoderOffsetDegrees_Swivels;
      break;
      case "NoNo":
        offset_FL = SwerveConstants.kFrontLeftMagEncoderOffsetDegrees_NoNo;
        offset_RL = SwerveConstants.kRearLeftMagEncoderOffsetDegrees_NoNo;
        offset_FR = SwerveConstants.kFrontRightMagEncoderOffsetDegrees_NoNo;
        offset_RR = SwerveConstants.kRearRightMagEncoderOffsetDegrees_NoNo;
        break;
      default:
        // Raise error!
    }
  
  m_frontLeft =
    new SwerveModuleOffboard(
      SwerveConstants.kFrontLeftDriveMotorPort,
      SwerveConstants.kFrontLeftTurningMotorPort,
      SwerveConstants.kFrontLeftMagEncoderPort,
      offset_FL);

  m_rearLeft =
    new SwerveModuleOffboard(
      SwerveConstants.kRearLeftDriveMotorPort,
      SwerveConstants.kRearLeftTurningMotorPort,
      SwerveConstants.kRearLeftMagEncoderPort,
      offset_RL);

  m_frontRight =
    new SwerveModuleOffboard(
      SwerveConstants.kFrontRightDriveMotorPort,
      SwerveConstants.kFrontRightTurningMotorPort,
      SwerveConstants.kFrontRightMagEncoderPort,
      offset_FR);

  m_rearRight =
    new SwerveModuleOffboard(
      SwerveConstants.kRearRightDriveMotorPort,
      SwerveConstants.kRearRightTurningMotorPort,
      SwerveConstants.kRearRightMagEncoderPort,
      offset_RR);

  // Odometry class for tracking robot pose
  m_odometry =
      new SwerveDriveOdometry(
          SwerveConstants.kDriveKinematics,
          Rotation2d.fromDegrees(m_imu.getYaw()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

          // Create and push Field2d to SmartDashboard.
    m_field = new Field2d();
    SmartDashboard.putData(m_field);

    // Configure the AutoBuilder last

    
     AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
          new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
          new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
          ConstantsOffboard.DRIVE_RPM_TO_METERS_PER_SECOND * ConstantsOffboard.kMaximumSparkMaxRPM, // Max module speed, in m/s
          Math.sqrt(SwerveConstants.kWheelBase*SwerveConstants.kWheelBase + SwerveConstants.kTrackWidth*SwerveConstants.kTrackWidth), // Drive base radius in meters. Distance from robot center to furthest module.
          new ReplanningConfig() // Default path replanning config. See the API for the options here
      ), // Reference to this subsystem to set requirements
      ()->{
        //Boolean Supplier that controls when the path will be mirrored for the red alliance
        //This wil flip the path being followed to the red side of the feild
        //THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
         return alliance.get() == DriverStation.Alliance.Red; 
        }
        return false;       
      },
      this
    );

    // Reference: https://www.chiefdelphi.com/t/has-anyone-gotten-pathplanner-integrated-with-the-maxswerve-template/443646

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));

    SmartDashboard.putData(m_field);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_imu.getYaw()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
    
    // Update robot position on Field2d.
    m_field.setRobotPose(getPose());

    pose_publisher.set(getPose());
    swerve_publisher.set(new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState() } );

    

    // Diagnostics

  if (Constants.kDebugLevel >=3) {

      SmartDashboard.putBoolean("DigitalInput", input.get());
      SmartDashboard.putBoolean("DigitalInputI", inputIR.get());

      SmartDashboard.putNumber("FL Mag Enc", m_frontLeft.getCanCoder());
      SmartDashboard.putNumber("FR Mag Enc", m_frontRight.getCanCoder());
      SmartDashboard.putNumber("RL Mag Enc", m_rearLeft.getCanCoder());
      SmartDashboard.putNumber("RR Mag Enc", m_rearRight.getCanCoder());

      SmartDashboard.putNumber("FL Drive Enc", m_frontLeft.getPosition().distanceMeters);
      SmartDashboard.putNumber("FR Drive Enc", m_frontRight.getPosition().distanceMeters);
      SmartDashboard.putNumber("RL Drive Enc", m_rearLeft.getPosition().distanceMeters);
      SmartDashboard.putNumber("RR Drive Enc", m_rearRight.getPosition().distanceMeters);

      SmartDashboard.putNumber("FL Disired Speed", m_frontLeft.getState().speedMetersPerSecond);
      SmartDashboard.putNumber("FL Actual Speed", m_frontLeft.getVelocity());
      SmartDashboard.putNumber("FL Drive Current", m_frontLeft.getCurrent());

      SmartDashboard.putNumber("FL Rot Speed", m_frontLeft.getState().angle.getDegrees());
      SmartDashboard.putNumber("FL Actual Rot", m_frontLeft.getAngle().getDegrees());
      SmartDashboard.putNumber("FL Drive Current",m_frontLeft.getTurnCurrent());
      
      SmartDashboard.putNumber("FL Turn Enc", m_frontLeft.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("FR Turn Enc", m_frontRight.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("RL Turn Enc", m_rearLeft.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("RR Turn Enc", m_rearRight.getPosition().angle.getDegrees());

      SmartDashboard.putNumber("Accel_X", accelerometer.getX());
      SmartDashboard.putNumber("Accel_Y", accelerometer.getY());
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_imu.getYaw()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rot", rot);

    // Apply joystick deadband
    xSpeed = MathUtil.applyDeadband(xSpeed, 0.1, 1.0);
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.1, 1.0);
    rot = MathUtil.applyDeadband(rot, 0.1, 1.0);

    var swerveModuleStates =
        SwerveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_imu.getYaw()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[SwerveConstants.kSwerveFL_enum]);
    m_frontRight.setDesiredState(swerveModuleStates[SwerveConstants.kSwerveFR_enum]);
    m_rearLeft.setDesiredState(swerveModuleStates[SwerveConstants.kSwerveRL_enum]);
    m_rearRight.setDesiredState(swerveModuleStates[SwerveConstants.kSwerveRR_enum]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[SwerveConstants.kSwerveFL_enum]);
    m_frontRight.setDesiredState(desiredStates[SwerveConstants.kSwerveFR_enum]);
    m_rearLeft.setDesiredState(desiredStates[SwerveConstants.kSwerveRL_enum]);
    m_rearRight.setDesiredState(desiredStates[SwerveConstants.kSwerveRR_enum]);
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    this.drive(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond,false);
    SmartDashboard.putNumber("DriveVelX", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("DriveVelY", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("DriveRotZ", speeds.omegaRadiansPerSecond);
  }
  
  public ChassisSpeeds getRobotRelativeSpeeds(){
    return SwerveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(),
                                                           m_frontRight.getState(),
                                                           m_rearLeft.getState(),
                                                           m_rearRight.getState());
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_imu.setYaw(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_imu.getYaw();
  }
}
