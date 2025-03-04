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
public class TeleopEjectCoralBringArmToCruiseElevatorDown extends SequentialCommandGroup {
  /** Creates a new TeleopEjectCoralBringArmToCruise. */
  public TeleopEjectCoralBringArmToCruiseElevatorDown() {
    /**
   * Per picture
   * Bottom button
   */
    addCommands(
      new IntakeShootCommand(),
      new ArmToPositionAndHold(ArmPositions.CoralCruise),
      new ElevatorToLevelAndHold(ElevatorHeights.ReefLevelOne),
      new StopElevator()
    );
  }
}
