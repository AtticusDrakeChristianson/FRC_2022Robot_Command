// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.GamepadAxisButton;
import edu.wpi.first.wpilibj.Joystick;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.DriveToDistanceBack;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FiveCargoAuto;
import frc.robot.commands.GreenLightCommand;
import frc.robot.commands.RainbowLightCommand;
import frc.robot.commands.SimpleAuto;
import frc.robot.commands.ThreeCargoAuto;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.reverseConveyor;
import frc.robot.commands.runIntake;
import frc.robot.commands.runIntakeServo;
import frc.robot.commands.runIntakeServoZero;
import frc.robot.commands.runLeftClimber;
import frc.robot.commands.runRightClimber;
import frc.robot.commands.stopConveyor;
import frc.robot.commands.reverseIntake;
import frc.robot.commands.runCannon;
import frc.robot.commands.runCannonServo;
import frc.robot.commands.runConveyor;
import frc.robot.subsystems.CannonServoSubsystem;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeServoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();
  private final CannonSubsystem m_cannonSubsystem = new CannonSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final LightSubsystem m_lightsSubsystem = new LightSubsystem();
  private final IntakeServoSubsystem m_intakeServoSubsystem = new IntakeServoSubsystem();
  private final CannonServoSubsystem m_cannonServoSubsystem = new CannonServoSubsystem();

  Joystick leftJoy = new Joystick(0);
  Joystick rightJoy = new Joystick(1);
  Joystick xbox = new Joystick(2);

  final double xboxLeftTrigger = xbox.getRawAxis(2);
  final double xboxRightTrigger = xbox.getRawAxis(3);
  final double xboxLeftStick = xbox.getRawAxis(1);
  final double xboxRightStick = xbox.getRawAxis(5);
  final JoystickButton xboxB = new JoystickButton(xbox, 2);
  final JoystickButton xboxX = new JoystickButton(xbox, 3);
  final JoystickButton xboxA = new JoystickButton(xbox, 1);
  final JoystickButton xboxRightBumper = new JoystickButton(xbox, 6);
  final JoystickButton xboxLeftBumper = new JoystickButton(xbox, 5);
  InternalButton xboxLeftTriggerButton = new InternalButton();
  Trigger leftTriggerDown = new Trigger();
  boolean condition0 = false;
  GamepadAxisButton xlt = new GamepadAxisButton(xbox, 2);
  GamepadAxisButton cannon0 = new GamepadAxisButton(xbox, 3);
  GamepadAxisButton climber0 = new GamepadAxisButton(xbox, 1);
  GamepadAxisButton climber1 = new GamepadAxisButton(xbox, 5);

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final runIntake m_runIntake = new runIntake(m_intakeSubsystem);
  private final reverseIntake m_reverseIntake = new reverseIntake(m_intakeSubsystem);
  private final runConveyor m_runConveyor = new runConveyor(m_conveyorSubsystem);
  private final reverseConveyor m_reverseConveyor = new reverseConveyor(m_conveyorSubsystem);
  private final runCannon m_runCannon = new runCannon(m_cannonSubsystem);
  private final runLeftClimber m_RunLeftClimber = new runLeftClimber(m_climberSubsystem);
  private final runRightClimber m_RunRightClimber = new runRightClimber(m_climberSubsystem);
  private final stopConveyor m_stopConveyor = new stopConveyor(m_conveyorSubsystem);
  private final DriveToDistance m_driveToDistance0 = new DriveToDistance(m_DriveSubsystem, 1, 0.5);
  private final DriveToDistanceBack m_driveToDistance1 = new DriveToDistanceBack(m_DriveSubsystem, -1, -0.5);
  private final SimpleAuto m_simpleAuto = new SimpleAuto(m_DriveSubsystem, m_intakeSubsystem, m_conveyorSubsystem, m_cannonSubsystem, m_intakeServoSubsystem);
  private final runIntakeServo m_runIntakeServo = new runIntakeServo(m_intakeServoSubsystem);
  private final runIntakeServoZero m_runIntakeServoZero = new runIntakeServoZero(m_intakeServoSubsystem);
  private final GreenLightCommand m_GreenLightCommand = new GreenLightCommand(m_lightsSubsystem);
  private final RainbowLightCommand m_RainbowLightCommand = new RainbowLightCommand(m_lightsSubsystem);
  private final ThreeCargoAuto m_threeCargoAuto = new ThreeCargoAuto(m_DriveSubsystem, m_intakeSubsystem, m_conveyorSubsystem, m_cannonSubsystem, m_intakeServoSubsystem);
  private final FiveCargoAuto m_fiveCargoAuto = new FiveCargoAuto(m_DriveSubsystem, m_intakeSubsystem, m_conveyorSubsystem, m_cannonSubsystem, m_intakeServoSubsystem);
  private final TurnToAngle m_TurnToAngle = new TurnToAngle(m_DriveSubsystem, 45, 0.7);
  private final runCannonServo m_cannonServo = new runCannonServo(m_cannonServoSubsystem);

  String trajectoryJSON = "paths/test.wpilib.json";
  Trajectory trajectory = new Trajectory();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_DriveSubsystem.setDefaultCommand(new DefaultDrive(m_DriveSubsystem, leftJoy::getY, rightJoy::getY));
    m_conveyorSubsystem.setDefaultCommand(new stopConveyor(m_conveyorSubsystem));
    m_intakeServoSubsystem.setDefaultCommand(new runIntakeServoZero(m_intakeServoSubsystem));
    m_lightsSubsystem.setDefaultCommand(m_GreenLightCommand);
    
  }

  
  public void SmartDashvoardContainer(){
    SmartDashboard.putNumber("Heading", m_DriveSubsystem.getHeading());
    SmartDashboard.putNumber("Avg Encoder", m_DriveSubsystem.getAverageEncoderDistance());
    SmartDashboard.putNumber("Encoder L", m_DriveSubsystem.seeLeftEncoders());
    SmartDashboard.putNumber("Encoder R", m_DriveSubsystem.seeRightEncoders());
    //SmartDashboard.putNumber("pose", m_DriveSubsystem.getPose());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    //Trigger intakeTrigger = new Trigger(() -> xbox.getRawAxis(2) > 0.65);
    //Trigger intakeTrigger = new Trigger(() -> xboxLeftTrigger > 0.65);
    Trigger shootTrigger = new Trigger(() -> xboxRightTrigger > 0.65);
    Trigger leftStick = new Trigger(() -> xboxLeftStick > 0.65);
    Trigger rightStick = new Trigger(() -> xboxRightStick > 0.65);
    Trigger exampleTrigger = new Trigger(() -> condition0 = true);
    
    xboxB.whileHeld(m_reverseIntake);
    //xboxB.whenReleased(m_reverseIntake.cancel());

    //intakeTrigger.whenActive(new runIntake(m_intakeSubsystem));
    xboxA.whileHeld(m_runIntakeServo);
    //xboxA.whenPressed(m_TurnToAngle);

    xboxRightBumper.whenPressed(m_runConveyor);
    xboxRightBumper.whenReleased(m_stopConveyor);
    xboxLeftBumper.whenPressed(m_reverseConveyor);
    xboxLeftBumper.whenReleased(m_stopConveyor);
    xboxX.toggleWhenPressed(m_cannonServo);

    xlt.whileHeld(m_runIntake);
    cannon0.whenHeld(m_runCannon);

    shootTrigger.whenActive(m_runCannon);

    climber0.whenHeld(m_RunLeftClimber);
    climber1.whenHeld(m_RunRightClimber);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    
    /*m_DriveSubsystem.resetEncoders();
    m_DriveSubsystem.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    m_DriveSubsystem.zeroHeading();

    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

    TrajectoryConfig config =
      new TrajectoryConfig(
        Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.kDriveKinematics)
                // Apply the voltage constraint
        
                .addConstraint(autoVoltageConstraint);
                config.setReversed(true);
    
    Trajectory exampleTrajectory =
      TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(-1, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-2, 0, new Rotation2d(0)),
        // Pass config
        config);
        /*try {
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
       } catch (IOException ex) {DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      }*/

        // Reset odometry to the starting pose of the trajectory.
    
    /*RamseteCommand ramseteCommand =
      new RamseteCommand(
        exampleTrajectory,
        m_DriveSubsystem::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(
        Constants.ksVolts,
        Constants.kvVoltSecondsPerMeter,
        Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        m_DriveSubsystem::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_DriveSubsystem::tankDriveVolts,
        m_DriveSubsystem);

        m_DriveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());
    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_DriveSubsystem.tankDrive(0, 0));*/
    
    // An ExampleCommand will run in autonomous
    return m_fiveCargoAuto;
  }
}
