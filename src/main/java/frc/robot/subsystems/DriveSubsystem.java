// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.util.Vector;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  StructPublisher<Pose2d> pose_publisher = NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose2d.struct).publish();
  StructArrayPublisher<SwerveModuleState> swerve_publisher = NetworkTableInstance.getDefault().getStructArrayTopic("Swerve States", SwerveModuleState.struct).publish();

  double offset_FL = 0;
  double offset_RL = 0;
  double offset_FR = 0;
  double offset_RR = 0;
  
  final Timer m_timerX = new Timer();
  final Timer m_timerY = new Timer();

  public double m_DriverSpeedScaleTran = 1.0;
  public double m_DriverSpeedScaleRot = 1.0;

  public double xVelocity = 0;
  public double yVelocity = 0;

  private double originalX = 0;
  private double originalY = 0;

  private boolean bigHitBoolean = false;

  // Robot swerve modules
  private final SwerveModuleOffboard m_frontLeft;
  private final SwerveModuleOffboard m_rearLeft;
  private final SwerveModuleOffboard m_frontRight;
  private final SwerveModuleOffboard m_rearRight;

  private double autoRotateSpeed = 0;

  Joystick m_Joystick = new Joystick(OIConstants.kDriverControllerPort);

  // The imu sensor
  public final Pigeon2 m_imu = new Pigeon2(14, "rio");
  public final LockOnTarget m_lock = new LockOnTarget();
  private final Vision2 m_Vision2;

  public boolean isAutoRotate = false;

  private double goalAngle = 0;
  private PIDController m_turnCtrl = new PIDController(0.03, 0.0065, 0.0035);
  private boolean roboNoSpino = true;
  private Timer rotationTimer = new Timer();

  public boolean isAutoRotateToggle = true;
  
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

  GenericEntry m_poseX =
  tab.add("Estimated Pose X", 0)
    .withPosition(10, 0)
    .withSize(2, 1)
    .getEntry();

  GenericEntry m_poseY =
  tab.add("Estimated Pose Y", 0)
    .withPosition(10, 1)
    .withSize(2, 1)
    .getEntry();

  GenericEntry m_poseRot =
  tab.add("Estimated Pose Rotation", 0)
    .withPosition(10, 2)
    .withSize(2, 1)
    .getEntry();

  GenericEntry m_GoalAngle =
  tab.add("Goal Angle", 0)
    .withPosition(10, 3)
    .withSize(2, 1)
    .getEntry();

  GenericEntry m_xSpeed =
  tab.add("X Speed", 0)
    .withPosition(10, 4)
    .withSize(2, 1)
    .getEntry();

  GenericEntry m_ySpeed =
  tab.add("Y Speed", 0)
    .withPosition(10, 5)
    .withSize(2, 1)
    .getEntry();

  GenericEntry m_rSpeed =
  tab.add("Spin Speed", 0)
    .withPosition(10, 6)
    .withSize(2, 1)
    .getEntry();

  GenericEntry m_hypot =
  tab.add("Hypot", 0)
    .withPosition(10, 7)
    .withSize(2, 1)
    .getEntry();

  GenericEntry m_alliance =
  tab.add("Alliance", "None")
    .withPosition(10, 8)
    .withSize(2, 1)
    .getEntry();

  ShuffleboardContainer m_pigeonValues = 
  tab.getLayout("Pigeon Values", BuiltInLayouts.kList)
    .withPosition(0, 0)
    .withSize(2, 3);

  ShuffleboardContainer m_frValues = 
  tab.getLayout("Front Right", BuiltInLayouts.kList)
    .withPosition(2, 0)
    .withSize(2, 3);

  ShuffleboardContainer m_flValues = 
  tab.getLayout("Front Left", BuiltInLayouts.kList)
    .withPosition(4, 0)
    .withSize(2, 3);

  ShuffleboardContainer m_brValues = 
  tab.getLayout("Front Right", BuiltInLayouts.kList)
    .withPosition(6, 0)
    .withSize(2, 3);

  ShuffleboardContainer m_blValues = 
  tab.getLayout("Front Right", BuiltInLayouts.kList)
    .withPosition(8, 0)    .withSize(2, 3);

    //Front Right Module Values
    GenericEntry m_frValuesAEA = m_frValues.add("Absolute Encoder Angle", 0).withSize(2,1).getEntry();
    GenericEntry m_frValuesTMA = m_frValues.add("Turn Motor Angle", 0).withSize(2, 1).getEntry();
    GenericEntry m_frValuesV = m_frValues.add("Velocity", 0).withSize(2, 1).getEntry();
    GenericEntry m_frValuesDC = m_frValues.add("Drive Current", 0).withSize(2, 1).getEntry();
    GenericEntry m_frValuesTC = m_frValues.add("Turn Current", 0).withSize(2, 1).getEntry();
    //Front Left Module Values
    GenericEntry m_flValuesAEA = m_flValues.add("Absolute Encoder Angle", 0).withSize(2,1).getEntry();
    GenericEntry m_flValuesTMA = m_flValues.add("Turn Motor Angle", 0).withSize(2, 1).getEntry();
    GenericEntry m_flValuesV = m_flValues.add("Velocity", 0).withSize(2, 1).getEntry();
    GenericEntry m_flValuesDC = m_flValues.add("Drive Current", 0).withSize(2, 1).getEntry();
    GenericEntry m_flValuesTC = m_flValues.add("Turn Current", 0).withSize(2, 1).getEntry();
    //Back Right Module Values
    GenericEntry m_brValuesAEA = m_brValues.add("Absolute Encoder Angle", 0).withSize(2,1).getEntry();
    GenericEntry m_brValuesTMA = m_brValues.add("Turn Motor Angle", 0).withSize(2, 1).getEntry();
    GenericEntry m_brValuesV = m_brValues.add("Velocity", 0).withSize(2, 1).getEntry();
    GenericEntry m_brValuesDC = m_brValues.add("Drive Current", 0).withSize(2, 1).getEntry();
    GenericEntry m_brValuesTC = m_brValues.add("Turn Current", 0).withSize(2, 1).getEntry();
    //Back Left Module Values
    GenericEntry m_blValuesAEA = m_blValues.add("Absolute Encoder Angle", 0).withSize(2,1).getEntry();
    GenericEntry m_blValuesTMA = m_blValues.add("Turn Motor Angle", 0).withSize(2, 1).getEntry();
    GenericEntry m_blValuesV = m_blValues.add("Velocity", 0).withSize(2, 1).getEntry();
    GenericEntry m_blValuesDC = m_blValues.add("Drive Current", 0).withSize(2, 1).getEntry();
    GenericEntry m_blValuesTC = m_blValues.add("Turn Current", 0).withSize(2, 1).getEntry();
    // Pigeon Values
    // m_pigeonValues.add("Yaw", m_imu.getYaw().getValueAsDouble());
    // m_pigeonValues.add("Roll", m_imu.getRoll().getValueAsDouble());
    // m_pigeonValues.add("Pitch", m_imu.getPitch().getValueAsDouble());
    // m_pigeonValues.add("Acceleration X", m_imu.getAccelerationX().getValueAsDouble());
    // m_pigeonValues.add("Acceleration Y", m_imu.getAccelerationY().getValueAsDouble());
    // m_pigeonValues.add("Acceleration Z", m_imu.getAccelerationZ().getValueAsDouble());
    // m_pigeonValues.add("Accumulated Gyro X", m_imu.getAccumGyroX().getValueAsDouble());
    // m_pigeonValues.add("Accumulated Gyro Y", m_imu.getAccumGyroY().getValueAsDouble());
    // m_pigeonValues.add("Accumulated Gyro Z", m_imu.getAccumGyroZ().getValueAsDouble());
    // m_pigeonValues.add("Pigeon Angular Velocity X", m_imu.getAngularVelocityXDevice().getValueAsDouble());
    // m_pigeonValues.add("Pigeon Angular Velocity Y", m_imu.getAngularVelocityYDevice().getValueAsDouble());
    // m_pigeonValues.add("Pigeon Angular Velocity Z", m_imu.getAngularVelocityZDevice().getValueAsDouble());
    // m_pigeonValues.add("World Angular Velocity X", m_imu.getAngularVelocityXWorld().getValueAsDouble());
    // m_pigeonValues.add("World Angular Velocity Y", m_imu.getAngularVelocityYWorld().getValueAsDouble());
    // m_pigeonValues.add("World Angluar Velocity Z", m_imu.getAngularVelocityZWorld().getValueAsDouble());
    // m_pigeonValues.add("Gravity Vector X", m_imu.getGravityVectorX().getValueAsDouble());
    // m_pigeonValues.add("Gravity Vector Y", m_imu.getGravityVectorY().getValueAsDouble());
    // m_pigeonValues.add("Gravity Vector Z", m_imu.getGravityVectorZ().getValueAsDouble());
    // m_pigeonValues.add("Magnetic Field X", m_imu.getMagneticFieldX().getValueAsDouble());
    // m_pigeonValues.add("Magnetic Field Y", m_imu.getMagneticFieldY().getValueAsDouble());
    // m_pigeonValues.add("Magnetic Field Z", m_imu.getMagneticFieldZ().getValueAsDouble());
    // m_pigeonValues.add("No Motion Count", m_imu.getNoMotionCount().getValueAsDouble());
    // m_pigeonValues.add("Quaternion W", m_imu.getQuatW().getValueAsDouble());
    // m_pigeonValues.add("Quaternion X", m_imu.getQuatX().getValueAsDouble());
    // m_pigeonValues.add("Quaternion Y", m_imu.getQuatY().getValueAsDouble());
    // m_pigeonValues.add("Quaternion Z", m_imu.getQuatZ().getValueAsDouble());
    // m_pigeonValues.add("Rate", m_imu.getRate());
    // m_pigeonValues.add("Raw Magnetic Field X", m_imu.getRawMagneticFieldX().getValueAsDouble());
    // m_pigeonValues.add("Raw Magnetic Field Y", m_imu.getRawMagneticFieldY().getValueAsDouble());
    // m_pigeonValues.add("Raw Magnetic Field Z", m_imu.getRawMagneticFieldZ().getValueAsDouble());
    // m_pigeonValues.add("Rotation 2D", m_imu.getRotation2d());
    // m_pigeonValues.add("Rotation 3D", m_imu.getRotation3d());
    // m_pigeonValues.add("Supply Voltage", m_imu.getSupplyVoltage().getValueAsDouble());
    // m_pigeonValues.add("Temperature", m_imu.getTemperature().getValueAsDouble());
    // m_pigeonValues.add("Up Time", m_imu.getUpTime().getValueAsDouble());
  // Create Field2d for robot and trajectory visualizations.
  public Field2d m_field; 
  // tab.add("Field", 0)
  //   .withPosition(0, 0)
  //   .withSize(8, 4)
  //   .getEntry();

  private String MyName;
  
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
  }

  private final SwerveDrivePoseEstimator m_poseEstimator;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(Vision2 m_Vision2) {
    m_turnCtrl.enableContinuousInput(-180, 180);
    m_turnCtrl.setTolerance(1.0);
    m_turnCtrl.reset();
    this.m_Vision2 = m_Vision2;
    MyName = Preferences.getString("RobotName", "NoDefault");
    System.out.println("Robot ID: " + MyName);
    switch(MyName) {
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
        System.out.println("Offsets loaded for NoNo");
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
          m_imu.getRotation2d(),
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
      this::getEstimatedPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
          new PIDConstants(7.0, 0.0, 0.0), // Translation PID constants
          new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
          4.5, // Max module speed, in m/s
          0.4, // Drive base radius in meters . Distance from robot center to furthest module.
          new ReplanningConfig()), // Default path replanning config. See the API for the options here
          ()->{
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
       // Reference to this subsystem to set requirements
    );

    // Reference: https://www.chiefdelphi.com/t/has-anyone-gotten-pathplanner-integrated-with-the-maxswerve-template/443646

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));

    SmartDashboard.putData(m_field);

    m_poseEstimator =
      new SwerveDrivePoseEstimator(
        Constants.SwerveConstants.kDriveKinematics,
        m_imu.getRotation2d(),
        getModulePositions(),
        getPose(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
    
    originalX = m_poseEstimator.getEstimatedPosition().getX();
    originalY = m_poseEstimator.getEstimatedPosition().getY();
    m_timerX.start();
    m_timerY.start();
    rotationTimer.start();
  }


  public void updateShuffleBoardValues(double xSpeed, double ySpeed, double rSpeed) {
    m_xSpeed.setDouble(xSpeed);
    m_ySpeed.setDouble(ySpeed);
    m_rSpeed.setDouble(rSpeed);
    m_poseX.setDouble(m_poseEstimator.getEstimatedPosition().getX());
    m_poseY.setDouble(m_poseEstimator.getEstimatedPosition().getY());
    m_poseRot.setDouble(m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    m_GoalAngle.setDouble(goalAngle);
    //Front Right Module Values
    m_frValuesAEA.setDouble(m_frontRight.getCanCoder());
    m_frValuesTMA.setDouble(m_frontRight.getAngle().getDegrees());
    m_frValuesV.setDouble(m_frontRight.getVelocity());
    m_frValuesDC.setDouble(m_frontRight.getCurrent());
    m_frValuesTC.setDouble(m_frontRight.getTurnCurrent());
    //Front Left Module Values
    m_flValuesAEA.setDouble(m_frontLeft.getCanCoder());
    m_flValuesTMA.setDouble(m_frontLeft.getAngle().getDegrees());
    m_flValuesV.setDouble(m_frontLeft.getVelocity());
    m_flValuesDC.setDouble(m_frontLeft.getCurrent());
    m_flValuesTC.setDouble(m_frontLeft.getTurnCurrent());
    //Back Right Module Values
    m_brValuesAEA.setDouble(m_rearRight.getCanCoder());
    m_brValuesTMA.setDouble(m_rearRight.getAngle().getDegrees());
    m_brValuesV.setDouble(m_rearRight.getVelocity());
    m_brValuesDC.setDouble(m_rearRight.getCurrent());
    m_brValuesTC.setDouble(m_rearRight.getTurnCurrent());
    //Back Left Module Values
    m_blValuesAEA.setDouble(m_rearLeft.getCanCoder());
    m_blValuesTMA.setDouble(m_rearLeft.getAngle().getDegrees());
    m_blValuesV.setDouble(m_rearLeft.getVelocity());
    m_blValuesDC.setDouble(m_rearLeft.getCurrent());
    m_blValuesTC.setDouble(m_rearLeft.getTurnCurrent());
    // Pigeon Values
    // m_pigeonValues.add("Yaw", m_imu.getYaw().getValueAsDouble());
    // m_pigeonValues.add("Roll", m_imu.getRoll().getValueAsDouble());
    // m_pigeonValues.add("Pitch", m_imu.getPitch().getValueAsDouble());
    // m_pigeonValues.add("Acceleration X", m_imu.getAccelerationX().getValueAsDouble());
    // m_pigeonValues.add("Acceleration Y", m_imu.getAccelerationY().getValueAsDouble());
    // m_pigeonValues.add("Acceleration Z", m_imu.getAccelerationZ().getValueAsDouble());
    // m_pigeonValues.add("Accumulated Gyro X", m_imu.getAccumGyroX().getValueAsDouble());
    // m_pigeonValues.add("Accumulated Gyro Y", m_imu.getAccumGyroY().getValueAsDouble());
    // m_pigeonValues.add("Accumulated Gyro Z", m_imu.getAccumGyroZ().getValueAsDouble());
    // m_pigeonValues.add("Pigeon Angular Velocity X", m_imu.getAngularVelocityXDevice().getValueAsDouble());
    // m_pigeonValues.add("Pigeon Angular Velocity Y", m_imu.getAngularVelocityYDevice().getValueAsDouble());
    // m_pigeonValues.add("Pigeon Angular Velocity Z", m_imu.getAngularVelocityZDevice().getValueAsDouble());
    // m_pigeonValues.add("World Angular Velocity X", m_imu.getAngularVelocityXWorld().getValueAsDouble());
    // m_pigeonValues.add("World Angular Velocity Y", m_imu.getAngularVelocityYWorld().getValueAsDouble());
    // m_pigeonValues.add("World Angluar Velocity Z", m_imu.getAngularVelocityZWorld().getValueAsDouble());
    // m_pigeonValues.add("Gravity Vector X", m_imu.getGravityVectorX().getValueAsDouble());
    // m_pigeonValues.add("Gravity Vector Y", m_imu.getGravityVectorY().getValueAsDouble());
    // m_pigeonValues.add("Gravity Vector Z", m_imu.getGravityVectorZ().getValueAsDouble());
    // m_pigeonValues.add("Magnetic Field X", m_imu.getMagneticFieldX().getValueAsDouble());
    // m_pigeonValues.add("Magnetic Field Y", m_imu.getMagneticFieldY().getValueAsDouble());
    // m_pigeonValues.add("Magnetic Field Z", m_imu.getMagneticFieldZ().getValueAsDouble());
    // m_pigeonValues.add("No Motion Count", m_imu.getNoMotionCount().getValueAsDouble());
    // m_pigeonValues.add("Quaternion W", m_imu.getQuatW().getValueAsDouble());
    // m_pigeonValues.add("Quaternion X", m_imu.getQuatX().getValueAsDouble());
    // m_pigeonValues.add("Quaternion Y", m_imu.getQuatY().getValueAsDouble());
    // m_pigeonValues.add("Quaternion Z", m_imu.getQuatZ().getValueAsDouble());
    // m_pigeonValues.add("Rate", m_imu.getRate());
    // m_pigeonValues.add("Raw Magnetic Field X", m_imu.getRawMagneticFieldX().getValueAsDouble());
    // m_pigeonValues.add("Raw Magnetic Field Y", m_imu.getRawMagneticFieldY().getValueAsDouble());
    // m_pigeonValues.add("Raw Magnetic Field Z", m_imu.getRawMagneticFieldZ().getValueAsDouble());
    // m_pigeonValues.add("Rotation 2D", m_imu.getRotation2d());
    // m_pigeonValues.add("Rotation 3D", m_imu.getRotation3d());
    // m_pigeonValues.add("Supply Voltage", m_imu.getSupplyVoltage().getValueAsDouble());
    // m_pigeonValues.add("Temperature", m_imu.getTemperature().getValueAsDouble());
    // m_pigeonValues.add("Up Time", m_imu.getUpTime().getValueAsDouble());
  }

  // private double newX = m_poseEstimator.getEstimatedPosition().getX();

  @Override
  public void periodic() {
    //updateShuffleBoardValues();
    // Update the odometry in the periodic block

    m_odometry.update(
        m_imu.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

    if (m_Vision2.getTarget().map((t) -> t.getPoseAmbiguity()).orElse(1.0) <= .2) { 
      m_Vision2.getRobotPose().ifPresent((robotPose) -> m_poseEstimator.addVisionMeasurement(robotPose, m_Vision2.getApriltagTime()));
    }

    m_poseEstimator.update(m_imu.getRotation2d(), getModulePositions());
    
    // Update robot position on Field2d.
    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    if (Constants.controllerMode == "j") {
      if (m_Joystick.getRawAxis(3) < 0) {
        m_DriverSpeedScaleTran = 1.0;
      }
      else {
        m_DriverSpeedScaleTran = Constants.kDriverSpeedLimitTran;
      }
    }

    pose_publisher.set(getPose());
    swerve_publisher.set(new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState() } ); // :3
    // Diagnostics

  if (Constants.kDebugLevel >=3) {
      SmartDashboard.putNumber("FL Mag Enc", m_frontLeft.getCanCoder());
      SmartDashboard.putNumber("FR Mag Enc", m_frontRight.getCanCoder());
      SmartDashboard.putNumber("RL Mag Enc", m_rearLeft.getCanCoder());
      SmartDashboard.putNumber("RR Mag Enc", m_rearRight.getCanCoder());

      SmartDashboard.putNumber("FL Angle State", m_frontLeft.getState().angle.getDegrees());
      SmartDashboard.putNumber("FL Angle SparkMax", m_frontLeft.getAngle().getDegrees());
      SmartDashboard.putNumber("FL Angle CanCoder", m_frontLeft.getCanCoder());
      SmartDashboard.putNumber("FL Angle Offset", m_frontLeft.getCanCoder() - m_frontLeft.getAngle().getDegrees());
      SmartDashboard.putNumber("FL Angle Current",m_frontLeft.getTurnCurrent());
      
      SmartDashboard.putNumber("FL Turn Enc", m_frontLeft.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("FR Turn Enc", m_frontRight.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("RL Turn Enc", m_rearLeft.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("RR Turn Enc", m_rearRight.getPosition().angle.getDegrees());
    }

  // SmartDashboard.putNumber("Estimated Pose X", m_poseEstimator.getEstimatedPosition().getX());
  // SmartDashboard.putNumber("Estimated Pose Y", m_poseEstimator.getEstimatedPosition().getY());
  // SmartDashboard.putNumber("Estimated Pose Rotation", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
  // SmartDashboard.putBoolean("isAutoRotate", isAutoRotate);

  // SmartDashboard.putNumber("Robot Velocity X", xVelocity);
  // SmartDashboard.putNumber("Robot Velocity Y", yVelocity);

  if (Math.abs(m_imu.getAccelerationX().getValueAsDouble()) > 1) {
    bigHitBoolean = true;
  }
  else if (Math.abs(m_imu.getAccelerationY().getValueAsDouble())> 1) {
    bigHitBoolean = true;
  }
  else {
    bigHitBoolean = false;
  }
  SmartDashboard.putBoolean("Big Hit", bigHitBoolean);
  SmartDashboard.putNumber("Gyro Acc Z", m_imu.getAccelerationZ().getValueAsDouble());

  Vector<Double> robotVector = new Vector<>();
  if (Math.abs(xVelocity) <= 0.1) {
    robotVector.add(xVelocity);
  }
  else {
    robotVector.add(0.0);
  }
  if (Math.abs(yVelocity) <= 0.1) {
    robotVector.add(yVelocity);
  }
  else {
    robotVector.add(0.0);
  }

  if (isAutoRotate) {
    autoRotateSpeed = m_lock.execute(m_poseEstimator.getEstimatedPosition(), robotVector);
  }

  if (isAutoRotate && isAutoRotateToggle) {
    m_lock.initialize();
    isAutoRotateToggle = false;
  }
  else if (!isAutoRotate) {
    isAutoRotateToggle = true;
  }

  getRobotVelocityX();
  getRobotVelocityY();
}

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void zeroIMU() {
    m_imu.setYaw(0.0);
    m_poseEstimator.resetPosition(m_imu.getRotation2d(), getModulePositions(), getPose());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_imu.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
    m_poseEstimator.resetPosition(m_imu.getRotation2d(), getModulePositions(), pose);
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
    updateShuffleBoardValues(xSpeed, ySpeed, rot);
    rot = isAutoRotate ? autoRotateSpeed : rot;
    // SmartDashboard.putNumber("xSpeed", xSpeed);
    // SmartDashboard.putNumber("ySpeed", ySpeed);
    // SmartDashboard.putNumber("rot", rot);
    //Square inputs
    // xSpeed=Math.signum(xSpeed)* xSpeed*xSpeed;
    // ySpeed=Math.signum(ySpeed)* ySpeed*ySpeed;
    // rot=Math.signum(rot)* rot*rot;

    // Apply joystick deadband
    xSpeed = MathUtil.applyDeadband(xSpeed, OIConstants.kDeadband, 1.0);
    ySpeed = MathUtil.applyDeadband(ySpeed, OIConstants.kDeadband, 1.0);
    rot = isAutoRotate ? rot : MathUtil.applyDeadband(rot, OIConstants.kDeadband, 1.0);

    // Apply speed scaling
    xSpeed = xSpeed * m_DriverSpeedScaleTran;
    ySpeed = ySpeed * m_DriverSpeedScaleTran;
    rot = rot * m_DriverSpeedScaleRot;

    if (isAutoRotate == false && Math.abs(rot / ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND) <= 0.1 && rotationTimer.hasElapsed(0.1)) {
      if (roboNoSpino) {
        goalAngle = m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
        roboNoSpino = false;
      }
      m_turnCtrl.setSetpoint(goalAngle);
      double m_output = MathUtil.clamp(m_turnCtrl.calculate(m_poseEstimator.getEstimatedPosition().getRotation().getDegrees()), -1.0, 1.0);
      rot = m_output;
    }
    else {
      goalAngle = m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
      m_turnCtrl.reset();
      roboNoSpino = true;
      rotationTimer.restart();
    }
    //SmartDashboard.putNumber("Goal Angle", goalAngle);

    var swerveModuleStates =
        SwerveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_imu.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds( swerveModuleStates, SwerveConstants.kMaxSpeedMetersPerSecond);
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
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[SwerveConstants.kSwerveFL_enum]);
    m_frontRight.setDesiredState(desiredStates[SwerveConstants.kSwerveFR_enum]);
    m_rearLeft.setDesiredState(desiredStates[SwerveConstants.kSwerveRL_enum]);
    m_rearRight.setDesiredState(desiredStates[SwerveConstants.kSwerveRR_enum]);
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    this.drive(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond,false);
    // SmartDashboard.putNumber("DriveVelX", speeds.vxMetersPerSecond);
    // SmartDashboard.putNumber("DriveVelY", speeds.vyMetersPerSecond);
    // SmartDashboard.putNumber("DriveRotZ", speeds.omegaRadiansPerSecond);
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

  /* Sets how fast the human driver can drive */
  public void setMaxOutput(double tran, double rot) {
    m_DriverSpeedScaleTran = tran;
    m_DriverSpeedScaleRot = rot;
  }

  public void toggleMaxOutput() {
    if (m_DriverSpeedScaleTran == 1.0){
      m_DriverSpeedScaleTran = Constants.kDriverSpeedLimitTran;
      m_DriverSpeedScaleRot = Constants.kDriverSpeedLimitRot;
    } else {
      m_DriverSpeedScaleTran = 1.0;
      m_DriverSpeedScaleRot = 1.0;
    }
  }

  public Pose2d getEstimatedPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public double getEstimatedPoseHyp() {
    var alliance = DriverStation.getAlliance();
    if (alliance.get() == DriverStation.Alliance.Red) {
      m_alliance.setString("Red");
      m_hypot.setDouble(Math.hypot(16.459 - m_poseEstimator.getEstimatedPosition().getX(), m_poseEstimator.getEstimatedPosition().getY()) - 5.55);
      return Math.hypot(16.459 - m_poseEstimator.getEstimatedPosition().getX(), m_poseEstimator.getEstimatedPosition().getY() - 5.55);
    }
    else {
      if (alliance.get() == DriverStation.Alliance.Blue) {
        m_alliance.setString("Blue");
      } else {
        m_alliance.setString("None");
      }
      m_hypot.setDouble(Math.hypot(m_poseEstimator.getEstimatedPosition().getX(), m_poseEstimator.getEstimatedPosition().getY()) - 5.55);
      return Math.hypot(m_poseEstimator.getEstimatedPosition().getX(), m_poseEstimator.getEstimatedPosition().getY() - 5.55);
    }
  }

  public void getRobotVelocityX() {
    if (m_timerX.hasElapsed(0.1)) {
      double newX = m_poseEstimator.getEstimatedPosition().getX();
      xVelocity = (newX - originalX) / m_timerX.get();
      originalX = newX;
      m_timerX.restart();
    }
  }

  public void getRobotVelocityY() {
    if (m_timerY.hasElapsed(0.1)) {
      double newY = m_poseEstimator.getEstimatedPosition().getY();
      yVelocity = (newY - originalY) / m_timerY.get();
      originalY = newY;
      m_timerY.restart();
    }
  }

  /**
   * @param pose The x and y position the robot thinks it's at.
   */
  public void setEstimatedPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_imu.getRotation2d(), getModulePositions(), pose);
  }
}