// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class CannonSubsystem extends SubsystemBase {

  TalonFX cannonMotor0 = new TalonFX(4);
  TalonFX cannonMotor1 = new TalonFX(5);
  /** Creates a new ExampleSubsystem. */
  public CannonSubsystem() {}

  public void runCannon(){
    cannonMotor0.set(ControlMode.PercentOutput, -0.4);
    cannonMotor1.set(ControlMode.PercentOutput, 0.4);
  }
  public void reverseCannon(){
    cannonMotor0.set(ControlMode.PercentOutput, 0.2);
    cannonMotor1.set(ControlMode.PercentOutput, -0.2);
  }
  public void stopCannon(){
    cannonMotor0.set(ControlMode.PercentOutput, 0);
    cannonMotor1.set(ControlMode.PercentOutput, 0);
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
