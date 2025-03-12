// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmPositions;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorHeights;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorAllTheWayDown extends SequentialCommandGroup {
  /** Creates a new ElevatorAllTheWayDown. */
  public ElevatorAllTheWayDown() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("*** Elevator down"),
      new ArmToPositionAndHold(ArmPositions.CoralCruise)
        .raceWith(new WaitCommand(1.0)),
      new DeferredCommand(
          () -> new ConditionalCommand(
            new PrintCommand("* Elevator already down"),
            new ElevatorToLevelAndHold(ElevatorHeights.ReefLevelOne), // if elevator is NOT down
            RobotContainer.elevatorSubsystem::isDown
          )
        , Set.of()),
      new StopElevator(),
      new PrintCommand("*** Elevator down - end")
    );
  }
}
