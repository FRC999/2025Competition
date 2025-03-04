// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStraightrajectoryToPickupStation extends SequentialCommandGroup {
  /** Creates a new AutoStraightrajectoryToPickupStation. */
  public AutoStraightrajectoryToPickupStation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("Button pressed start"),
        new AutonomousTrajectory2Poses(new Pose2d(6.252, 2.500, new Rotation2d()), new Pose2d(6.252, 3.860, new Rotation2d())),
        new AutonomousTrajectory2Poses(new Pose2d(6.252, 3.860, new Rotation2d()), new Pose2d(1.437, 0.919, new Rotation2d())),
        new IntakeCoralAndMoveToCruisePositionSequence(),
        new AutonomousTrajectory2Poses(new Pose2d(3.450, 2.621, new Rotation2d()), new Pose2d(1.437, 0.919, new Rotation2d())),
        new WaitCommand(0.2),
        new CoralPlaceOnFour(),
        new ElevatorAllTheWayDown(),
        new AutonomousTrajectory2Poses(new Pose2d(1.437, 0.919, new Rotation2d()), new Pose2d(6.252, 3.860, new Rotation2d())),
        new PrintCommand("Button pressed end")
        //new ElevatorToLevelForCoralPlacement(ElevatorHeights.ReefLevelFour, ArmPositions.ReefLevelFour),

    );
  }
}
