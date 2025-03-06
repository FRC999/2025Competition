// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveCloserToReef extends SequentialCommandGroup {
  DoubleSupplier timeToDrive;
  BooleanSupplier doWeNeedToDrive;
  
  /** Creates a new DriveCloserToReef. */
  public DriveCloserToReef(DoubleSupplier ttr, BooleanSupplier dndrive) {
    timeToDrive = ttr;
    doWeNeedToDrive = dndrive;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DeferredCommand( () ->
        new ConditionalCommand( new PanForwardOrBackWithLowSpeed(0.3).raceWith(new WaitCommand(timeToDrive.getAsDouble())),
          new PrintCommand("== Do not Need To Drive forward"),
          doWeNeedToDrive ),
        Set.of()
      )
    );
  }
}
