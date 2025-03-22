// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestBluCage extends SequentialCommandGroup {
  /** Creates a new TestBluCage. */
  public TestBluCage() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoDriveWithPP("Blu-BargeToReef11"),
      new TeleopMoveToL4RotateArm(),
      new TeleopEjectCoralBringArmToCruise(),
      new ElevatorAllTheWayDown(),
      new AutoDriveWithPP("Blu-Reef11ToCoralTop"),
      new TeleopCoralIntakeSequence(),
      new AutoDriveWithPP("Blu-CoralTopToReef9"),
      //new TeleopMoveToL4RotateArm(),
      new TeleopEjectCoralBringArmToCruise(),
      new ElevatorAllTheWayDown(),
      new AutoDriveWithPP("Blu-Reef9toCoralTop"),
      new TeleopCoralIntakeSequence(),
      new AutoDriveWithPP("Blu-CoralTopToReef10"),
      //new TeleopMoveToL4RotateArm(),
      new TeleopEjectCoralBringArmToCruise()
    );
  }
}
