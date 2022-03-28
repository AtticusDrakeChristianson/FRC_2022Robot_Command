// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class DriveSubsystem extends SubsystemBase {
    WPI_TalonSRX _leftFront = new WPI_TalonSRX(0);
    WPI_TalonSRX _leftRear = new WPI_TalonSRX(1);
    WPI_TalonSRX _rightFront = new WPI_TalonSRX(2);
    WPI_TalonSRX _rightRear = new WPI_TalonSRX(3);
    MotorControllerGroup _leftDrive = new MotorControllerGroup(_leftFront, _leftRear);
    MotorControllerGroup _rightDrive = new MotorControllerGroup(_rightFront, _rightRear);
    DifferentialDrive _diffDrive = new DifferentialDrive(_leftDrive, _rightDrive);
    WPI_PigeonIMU _pidgey = new WPI_PigeonIMU(_rightRear);
    private final DifferentialDriveOdometry m_odometry;
    private double m_yaw;
  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {

    _leftDrive.setInverted(true);
    _leftFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    _rightFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    m_odometry = new DifferentialDriveOdometry(_pidgey.getRotation2d());
  }

  /*public double getYaw(){
    double ypr[] = {0, 0, 0};
    _pidgey.getYawPitchRoll(ypr);
    return ypr[0];
}*/
  public void tankDrive(double lft, double rgt) {
    _diffDrive.tankDrive(lft, rgt);
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(((0.4788/4096)*(_leftFront.getSelectedSensorPosition())*10), -(((0.4788/4096)*(_rightFront.getSelectedSensorPosition())) * 10));
  }
  
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, _pidgey.getRotation2d());
  }

  public void arcadeDrive(double fwd, double rot) {
    _diffDrive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    _leftDrive.setVoltage(leftVolts);
    _rightDrive.setVoltage(rightVolts);
    _diffDrive.feed();
  }

  public void resetEncoders() {
    _leftFront.setSelectedSensorPosition(0);
    _rightFront.setSelectedSensorPosition(0);
  }

  public double seeLeftEncoders() {
    return (_leftFront.getSelectedSensorPosition() * (0.4788/4096));
  }
  public double seeRightEncoders() {
    return -(_rightFront.getSelectedSensorPosition() * (0.4788/4096));
  }

  public double getAverageEncoderDistance() {
    return ((seeLeftEncoders() + seeRightEncoders()) / 2.0);
  }

  public void setMaxOutput(double maxOutput) {
    _diffDrive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    _pidgey.reset();
  }

  public double getHeading() {
    return _pidgey.getRotation2d().getDegrees();
}

 /* public double getTurnRate() {
    return -_pidgey.getRate();
  }*/

  @Override
  public void periodic() {
    m_odometry.update(
        _pidgey.getRotation2d(), (0.4788/4096)*(_leftFront.getSelectedSensorPosition()), -((0.4788/4096)*(_rightFront.getSelectedSensorPosition())));
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
