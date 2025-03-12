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
import frc.robot.Constants.GPMConstants.ArmConstants;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorHeights;
import frc.robot.Constants.GPMConstants.IntakeConstants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCoralAndMoveToCruisePositionSequence extends SequentialCommandGroup {
  /** Creates a new IntakeCoralAndMoveToCruisePositionSequence. */
  public IntakeCoralAndMoveToCruisePositionSequence() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("*** Intake Coral sequence start"),
      // Reason - if we're already all the way down, I do not need to rotate the arm to CoralCruise first
      new DeferredCommand(
        () -> new ConditionalCommand(
          new PrintCommand("* Elevator already down"),
          new ElevatorAllTheWayDown(), // if elevator is NOT down
          RobotContainer.elevatorSubsystem::isDown
        )
        , Set.of()),
      new PrintCommand("* elevator down"),
      new ArmToPositionAndHold(ArmConstants.ArmPositions.CoralIntake)
        .raceWith(new WaitCommand(1)) // We observed this command not ending on its own sometimes, possibly because of the chain slack
      ,
      new PrintCommand("* arm at intake position"),
      new IntakeCoralCommand(IntakeConstants.coralIntakePower),
      new PrintCommand("* coral in intake"),
      new ArmToPositionAndHold(ArmConstants.ArmPositions.CoralCruise),
      new PrintCommand("* arm at cruise position"),
      new PrintCommand("*** Intake Coral sequence end")
    );
  }
}
