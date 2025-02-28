// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmPositions;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorHeights;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRed20PointsLeft extends SequentialCommandGroup {
  /** Creates a new AutoRed20PointsRight. */
  public AutoRed20PointsLeft() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    try {
      addCommands(
          new RunTrajectorySequenceRobotAtStartPoint("Red-BargeToReef1"),
          new ElevatorToLevelForCoralPlacement(ElevatorHeights.ReefLevelFour, ArmPositions.ReefLevelFour),
          new IntakeCoralOutCommand(0.2),
          new ElevatorToLevelForCoralPlacement(ElevatorHeights.Down, ArmPositions.CoralCruise),
          new RunTrajectorySequenceRobotAtStartPoint("Red-Reef1toCoralBottom"),
          new ElevatorToLevelForCoralPlacement(ElevatorHeights.CoralIntake, ArmPositions.CoralIntake),
          new IntakeCoralCommand(0.2),
          new ElevatorToLevelForCoralPlacement(ElevatorHeights.Down, ArmPositions.CoralCruise),
          new RunTrajectorySequenceRobotAtStartPoint("Red-CoralBottomToReef9"),
          new ElevatorToLevelForCoralPlacement(ElevatorHeights.ReefLevelFour, ArmPositions.ReefLevelFour),
          new IntakeCoralOutCommand(0.2),
          new ElevatorToLevelForCoralPlacement(ElevatorHeights.AlgaeReefLow, ArmPositions.AlgaeIntake),
          new IntakeAlgaeRollerInAndHold(),
          new ElevatorToLevelForCoralPlacement(ElevatorHeights.Down, ArmPositions.Processor),
          new RunTrajectorySequenceRobotAtStartPoint("Red-Reef9ToProcessor"),
          new ElevatorToLevelForCoralPlacement(ElevatorHeights.Processor, ArmPositions.Processor),
          new IntakeAlgaeRollOutCommand()
      );
    } catch (Exception e) {
      System.out.println("***Error: " + e + "! ***");
    }
  }
}
