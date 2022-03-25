// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Servo;

public class IntakeSubsystem extends SubsystemBase {
    TalonSRX intakeMotor0 = new TalonSRX(7);
    Servo intakeServo0 = new Servo(1);

  public void runIntake(){
      intakeMotor0.set(ControlMode.PercentOutput, 0.35);
  }
  public void reverseIntake(){
    intakeMotor0.set(ControlMode.PercentOutput, -0.35);
  }
  public void stopIntake(){
    intakeMotor0.set(ControlMode.PercentOutput, 0);
  }
  public void intakeServoForward(){
    intakeServo0.setAngle(0);
  }
  public void intakeServoReverse(){
    intakeServo0.setAngle(90);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
