// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TeleopCoralIntakeSequence extends SequentialCommandGroup {
  /** Creates a new TeleopCoralIntakeSequence. */
  public TeleopCoralIntakeSequence() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Reason - if we're already all the way down, I do not need to rotate the arm to CoralCruise first
      new DeferredCommand(
        () -> new ConditionalCommand(
          new PrintCommand("* Elevator already down"),
          new ElevatorAllTheWayDown(), // if elevator is NOT down
          RobotContainer.elevatorSubsystem::isDown
        )
        , Set.of()),
      new IntakeCoralAndMoveToCruisePositionSequence()
    );
  }
}
