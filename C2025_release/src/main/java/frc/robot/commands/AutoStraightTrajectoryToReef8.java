// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmPositions;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorHeights;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStraightTrajectoryToReef8 extends SequentialCommandGroup {
  /** Creates a new AutoStraightTrajectoryToReef8. */
  public AutoStraightTrajectoryToReef8() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    try {
      addCommands(
        new PrintCommand("Button pressed start"),
        new AutonomousTrajectory2Poses(new Pose2d(3.66, 1.500, new Rotation2d()), new Pose2d(8.000, 1.500, new Rotation2d())),
        new AutonomousTrajectory2Poses(new Pose2d(3.660, 4.160, new Rotation2d()), new Pose2d(3.660, 1.500, new Rotation2d())),
        // new RunTrajectorySequenceRobotAtStartPoint("Blu-BargeToReef8-PartOne"),
        // new RunTrajectorySequenceRobotAtStartPoint("Blu-BargeToReef8-PartTwo"),
        new PrintCommand("Button pressed end")
        //new ElevatorToLevelForCoralPlacement(ElevatorHeights.ReefLevelFour, ArmPositions.ReefLevelFour),
        //new IntakeCoralOutCommand(0.2)
      );
    } catch (Exception e) {
      System.out.println("***Error: " + e + "! ***");
    }
  }
}
