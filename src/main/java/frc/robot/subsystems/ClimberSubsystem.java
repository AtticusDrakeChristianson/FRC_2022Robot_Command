// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class ClimberSubsystem extends SubsystemBase {
  TalonSRX climbMotor0 = new TalonSRX(8);
  TalonSRX climbMotor1 = new TalonSRX(9);

  public void runLeftClimber(){
    climbMotor0.set(ControlMode.PercentOutput, -0.55);
  }
  public void reverseLeftClimber(){
    climbMotor0.set(ControlMode.PercentOutput, 0.55);
  }
  public void stopLeftClimber(){
    climbMotor0.set(ControlMode.PercentOutput, 0);
  }
  public void runRightClimber(){
    climbMotor1.set(ControlMode.PercentOutput, -0.55);
}
public void reverseRightClimber(){
    climbMotor1.set(ControlMode.PercentOutput, 0.55);
}
public void stopRightClimber(){
    climbMotor1.set(ControlMode.PercentOutput, 0);
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
