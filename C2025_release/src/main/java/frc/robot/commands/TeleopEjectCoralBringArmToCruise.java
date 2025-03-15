// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmPositions;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorHeights;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TeleopEjectCoralBringArmToCruise extends SequentialCommandGroup {
  /** Creates a new TeleopEjectCoralBringArmToCruise. */
  public TeleopEjectCoralBringArmToCruise() {
    /**
   * Per picture
   * Bottom button
   */
    addCommands(
      // If shooting coral when elevator is down, gently place it on L1
      new DeferredCommand(
          () -> new ConditionalCommand(
            new PlaceCoralGentlyInL1(),
            new IntakeShootCommand(), // if elevator is NOT down
            RobotContainer.elevatorSubsystem::isDown
          )
        , Set.of()),
      new ArmToPositionAndHold(ArmPositions.CoralCruise),
      new DeferredCommand(
          () -> new ConditionalCommand(
            new PrintCommand("* Elevator already down"),
            new ElevatorToLevelAndHold(ElevatorHeights.ReefLevelOne), // if elevator is NOT down
            RobotContainer.elevatorSubsystem::isDown
          )
        , Set.of())
      //new StopElevator()
    );
  }
}
