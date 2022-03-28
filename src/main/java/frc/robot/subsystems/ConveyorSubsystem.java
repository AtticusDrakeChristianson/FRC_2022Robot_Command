// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class ConveyorSubsystem extends SubsystemBase {

  TalonFX conveyorMotor0 = new TalonFX(6);
  /** Creates a new ExampleSubsystem. */
  public ConveyorSubsystem() {}

  public void runConveyor(){
    conveyorMotor0.set(ControlMode.PercentOutput, -1);
  }
  public void runConveyorSlow(){
    conveyorMotor0.set(ControlMode.PercentOutput, -0.3);
  }
  public void reverseConveyor(){
    conveyorMotor0.set(ControlMode.PercentOutput, 1);
  }
  public void stopConveyor(){
    conveyorMotor0.set(ControlMode.PercentOutput, 0);
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
