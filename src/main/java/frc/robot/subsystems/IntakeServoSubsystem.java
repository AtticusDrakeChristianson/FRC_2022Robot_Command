// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeServoSubsystem extends SubsystemBase {
    Servo intakeServo0 = new Servo(1);
  /** Creates a new ExampleSubsystem. */
  public IntakeServoSubsystem() {}

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
