package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeServoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** A simple auto that drives back to collect a second cargo and scores two into the upper hub */
public class FiveCargoAuto extends SequentialCommandGroup {
  /**
   * Creates a new SimpleAuto.
   *
   * @param drive The drive subsystem this command will run on
   */
  public FiveCargoAuto(DriveSubsystem drive, IntakeSubsystem intake, ConveyorSubsystem conveyor, CannonSubsystem cannon, IntakeServoSubsystem servo) {
    addCommands(
        // Drive forward the specified distance
        new ParallelCommandGroup(new DriveToDistanceBack(drive, 0.9, 0.8),new runCannon(cannon).withTimeout(1.6), new SequentialCommandGroup(new WaitCommand(0.5), new ParallelCommandGroup(new runIntake(intake), new runConveyorSlow(conveyor)).withTimeout(1)), new runIntakeServo(servo).withTimeout(1)),
        new ParallelCommandGroup(new runConveyor(conveyor), new runCannon(cannon)).withTimeout(1.5),
        new WaitCommand(0.2),
        new DriveToDistance(drive, 0.12, 0.8),
        new TurnToAngleBack(drive, 83.25, 0.7),
        new ParallelCommandGroup(new DriveToDistanceBack(drive, 2.5, 0.8), new SequentialCommandGroup(new WaitCommand(0.7), new ParallelCommandGroup(new runIntake(intake), new runConveyorSlow(conveyor)).withTimeout(1.3))),
        new TurnToAngle(drive, 43, 0.7),
        new WaitCommand(0.25),
        new DriveToDistance(drive, 0.3, 0.8),
        new ParallelCommandGroup(new runConveyor(conveyor), new runCannon(cannon)).withTimeout(2),
        new WaitCommand(0.25),
        new TurnToAngleBack(drive, 8.2, 0.7),
        new ParallelCommandGroup(new DriveToDistanceBack(drive, 3.6, 0.8), new runIntake(intake).withTimeout(4), new runConveyor(conveyor).withTimeout(4)),
        new ParallelCommandGroup(new runIntake(intake), new runConveyorSlow(conveyor)).withTimeout(0.5),
        new DriveToDistance(drive, 2.6, 1),
        new DriveToDistance(drive, 1, 0.75),
        new ParallelCommandGroup(new TurnToAngle(drive, 8.2, 0.7), new runCannon(cannon).withTimeout(0.7)),
        new ParallelCommandGroup(new runConveyor(conveyor), new runCannon(cannon)).withTimeout(1.5)
    );
  }
}