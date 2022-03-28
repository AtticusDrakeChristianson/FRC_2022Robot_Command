package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeServoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** A simple auto that drives back to collect a second cargo and scores two into the upper hub */
public class SimpleAuto extends SequentialCommandGroup {
  /**
   * Creates a new SimpleAuto.
   *
   * @param drive The drive subsystem this command will run on
   */
  public SimpleAuto(DriveSubsystem drive, IntakeSubsystem intake, ConveyorSubsystem conveyor, CannonSubsystem cannon, IntakeServoSubsystem servo) {
    addCommands(
        // Drive forward the specified distance
        new ParallelCommandGroup(new DriveToDistanceBack(drive, 1, 0.7), new runIntake(intake).withTimeout(2), new runConveyor(conveyor).withTimeout(2), new runIntakeServo(servo).withTimeout(2)),
        new DriveToDistance(drive, 1, 0.7),
        new ParallelCommandGroup(new runConveyor(conveyor), new runCannon(cannon)).withTimeout(3)
    );
  }
}