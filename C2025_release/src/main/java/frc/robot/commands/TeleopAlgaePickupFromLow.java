// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GPMConstants.ArmConstants;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorHeights;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TeleopAlgaePickupFromLow extends SequentialCommandGroup {
  /** Creates a new TestElevatorAlgaePickupSequence. */
  public TeleopAlgaePickupFromLow() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new PrintCommand("*** Start of Algae Pickup ready from Low"), 
        new ArmToPositionAndHold(ArmConstants.ArmPositions.CoralCruise),
        new PrintCommand("Arm At cruise position"),
        new ElevatorToLevelAndHold(ElevatorHeights.AlgaeReefLow),
        new PrintCommand("Elevator at algae reef low"),
        new ArmToPositionAndHold(ArmConstants.ArmPositions.AlgaeIntake),
        new IntakeAlgaeRollerInAndHold(),
        new PrintCommand("Arm ready, rollers ON"),
        new PrintCommand("*** End of Algae Pickup Ready from Low")
    );
  }
}
