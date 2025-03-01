// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GPMConstants.ArmConstants;
import frc.robot.Constants.GPMConstants.IntakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCoralAndMoveToCruisePositionSequence extends SequentialCommandGroup {
  /** Creates a new IntakeCoralAndMoveToCruisePositionSequence. */
  public IntakeCoralAndMoveToCruisePositionSequence() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmToPositionAndHold(ArmConstants.ArmPositions.CoralIntake),
      new PrintCommand("arm at intake position"),
      new IntakeCoralCommand(0.2),
      new PrintCommand("arm in intake"),
      new ArmToPositionAndHold(ArmConstants.ArmPositions.CoralCruise),
      new PrintCommand("arm at cruise position")
    );
  }
}
