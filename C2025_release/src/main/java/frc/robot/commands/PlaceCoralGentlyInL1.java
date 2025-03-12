// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmPositions;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorHeights;
import frc.robot.RobotContainer;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceCoralGentlyInL1 extends SequentialCommandGroup {
  /** Creates a new PlaceCoralGentlyInL1. */
  public PlaceCoralGentlyInL1() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmToPositionAndHold(ArmPositions.CoralIntake),
      //new WaitCommand(0.5),
      new InstantCommand(() -> RobotContainer.intakeSubsystem.runIntake(0.25),RobotContainer.intakeSubsystem),
      new WaitCommand(1.5),
      new StopIntake(),
      new ArmToPositionAndHold(ArmPositions.CoralCruise)
    );
  }
}
