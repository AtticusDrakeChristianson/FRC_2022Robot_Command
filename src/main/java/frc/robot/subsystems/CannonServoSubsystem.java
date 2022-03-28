// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CannonServoSubsystem extends SubsystemBase {
    Servo cannonServo0 = new Servo(2);
  /** Creates a new ExampleSubsystem. */
  public CannonServoSubsystem() {}

  public void intakeServoForward(){
    cannonServo0.setAngle(0);
  }
  public void intakeServoReverse(){
    cannonServo0.setAngle(80);
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
