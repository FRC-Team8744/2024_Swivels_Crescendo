// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.VisionShoot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision2;
import frc.robot.commands.AmpShoot;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.IntakeSpinUpTeleop;
import frc.robot.commands.LockOnShooter;
import frc.robot.commands.OuttakeRun;
import frc.robot.commands.ShootRing;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final Vision2 m_Vision2 = new Vision2();
  public final DriveSubsystem m_robotDrive = new DriveSubsystem(m_Vision2);
  public final Intake m_intake = new Intake();
  public final Shooter m_shooter = new Shooter();
  public final Index m_index = new Index();
  public final LEDS m_leds = new LEDS();
  public final Climber m_climber = new Climber();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_Joystick = new Joystick(OIConstants.kDriverControllerPort);
  // XboxController m_codriverController = new XboxController(OIConstants.kCodriverControllerPort);
  CommandXboxController m_driver = new CommandXboxController(OIConstants.kDriverControllerPort);

  private AutoCommandManager m_autoManager = new AutoCommandManager(
      m_intake, 
      m_index, 
      m_shooter,
      m_leds,
      m_robotDrive,
      m_climber,
      m_Vision2);

  // A chooser for autonomous commands
  // private final SendableChooser<Command> m_autoChooser;

  // private final LockOnShooterAuto m_lockAuto = new LockOnShooterAuto(m_shooter.m_pivot, m_Vision2, m_robotDrive, m_shooter);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_leds.ledOn(0,0,255);
    if (Constants.PDH.getVoltage() < 12.3) {
        m_leds.setSlashLed(255, 0, 0);
    } else if (Constants.PDH.getVoltage() <= Preferences.getDouble("BatteryJuice", Constants.PDH.getVoltage()) + 0.2) {
        m_leds.setSlashLed(255, 20, 0);
    } else {
      m_leds.ledOn(0, 0, 255);
    }

    // Configure the button bindings
    configureButtonBindings();

  // Configure default commands
  if (Constants.controllerMode == "x") {
      m_robotDrive.setDefaultCommand(
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.
          new RunCommand(
              () ->
                  m_robotDrive.drive(
                      -m_driverController.getLeftY() * SwerveConstants.kMaxSpeedTeleop,
                      -m_driverController.getLeftX() * SwerveConstants.kMaxSpeedTeleop,
                      -m_driverController.getRightX() * ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND,
                      true),
              m_robotDrive));
  }
    // m_autoChooser = AutoBuilder.buildAutoChooser();  // Default auto will be 'Commands.none()'

    // SmartDashboard.putData("Auto Mode", m_autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  // Command lockOnTargetCommand = new LockOnTarget(m_robotDrive);
  LockOnShooter lockOnShooter = new LockOnShooter(m_shooter.m_pivot, m_Vision2, m_robotDrive, m_shooter, m_leds);
  private void configureButtonBindings() {
    if (Constants.controllerMode == "x") {
      // new JoystickButton(m_driverController, Button.kB.value)
      // .onTrue(new auto_led(m_Vision2, m_robotDrive, m_leds, m_shooter, m_shooter.m_pivot).withTimeout(2.0));

      new JoystickButton(m_driverController, Button.kRightStick.value)
      .toggleOnTrue(Commands.runOnce(() -> m_robotDrive.isAutoRotate = !m_robotDrive.isAutoRotate)
      .alongWith(Commands.runOnce(() -> lockOnShooter.toggle())));
      // .toggleOnTrue(Commands.runOnce(() -> lockOnShooter.toggle()));
      // .alongWith(new RevShooter(m_shooter, m_index, 3500))));
      // .toggleOnFalse(Commands.runOnce(() -> m_robotDrive.isAutoRotate = false));

      m_driver.leftTrigger().whileTrue(new AmpShoot(m_climber, m_shooter, m_index, m_leds));
      m_driver.rightTrigger().whileTrue(new ShootRing(m_shooter, m_index, m_leds));
      new JoystickButton(m_driverController, Button.kLeftBumper.value)
      .whileTrue(new IntakeSpinUpTeleop(m_intake, m_shooter, m_index, m_leds, m_shooter.m_pivot, m_Vision2, m_robotDrive, m_driverController, lockOnShooter));

      new JoystickButton(m_driverController, Button.kRightBumper.value)
      .whileTrue(new VisionShoot(m_shooter, m_index, m_leds, m_Vision2, m_shooter.m_pivot, m_robotDrive)
      .finallyDo(() -> {
        m_robotDrive.isAutoRotate = false; 
        lockOnShooter.reset();
      }));
     // .andThen(Commands.runOnce(() -> m_robotDrive.isAutoRotate = !m_robotDrive.isAutoRotate)
     // .alongWith(Commands.runOnce(() ->  lockOnShooter.toggle()))));
      
      new JoystickButton(m_driverController, Button.kX.value)
      .whileTrue(new OuttakeRun(m_intake, m_shooter, m_index));
      new JoystickButton(m_driverController, Button.kY.value)
      .whileTrue(new ClimbUp(m_climber));
      new JoystickButton(m_driverController, Button.kA.value)
      .whileTrue(new ClimbDown(m_climber));
      new POVButton(m_driverController, 180)
      .whileTrue(new InstantCommand(() -> m_shooter.stopShooter()));

      new POVButton(m_driverController, 0)
      .whileTrue(new IntakeRun(m_intake, m_shooter, m_index, m_leds));
      new JoystickButton(m_driverController, Button.kBack.value)
      .whileTrue(new RunCommand(() -> m_robotDrive.zeroIMU()));
      new JoystickButton(m_driverController, Button.kLeftStick.value)
      .toggleOnTrue(Commands.runOnce(() -> m_robotDrive.toggleMaxOutput()));
      new JoystickButton(m_driverController, Button.kStart.value)
      .toggleOnTrue(Commands.runOnce(() -> {m_robotDrive.isAutoRotate = false; lockOnShooter.reset(); m_autoManager.m_lockAuto.reset();}));
    } 
  }

  public Command getAutonomousCommand() {
    Command autoCommand = m_autoManager.getAutoManagerSelected();
    return autoCommand;
  }
}