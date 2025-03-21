// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestElevatorToL4AndHold extends SequentialCommandGroup {
  /** Creates a new TestElevatorToL4AndHold. */
  public TestElevatorToL4AndHold() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
       new DeferredCommand(
        () -> new PrintCommand("****SU"+System.currentTimeMillis())
        , Set.of()),
      new TeleopMoveToL4RotateArm(),
      new DeferredCommand(
        () -> new PrintCommand("****EU"+System.currentTimeMillis())
        , Set.of())
    );
  }
}
