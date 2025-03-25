// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBluCageWithTestTrajectories extends SequentialCommandGroup {
  /** Creates a new AutoBluCageWithTestTrajectories. */
  public AutoBluCageWithTestTrajectories() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("Test-Blu-BargeToReef11", false, false),
      new WaitCommand(0.5),
      new TeleopMoveToL4RotateArm(),
      new TeleopEjectCoralBringArmToCruise(),
      new ElevatorAllTheWayDown(),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("Test-Blu-Reef11ToCoralTop", false, false),
      new AutoIntakeSequenceCoral(),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("Test-Blu-CoralTopToReef9", false, false),
      new TeleopMoveToL4RotateArm(),
      new TeleopEjectCoralBringArmToCruise(),
      new ElevatorAllTheWayDown(),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("Test-Blu-Reef9toCoralTop", false, false),
      new AutoIntakeSequenceCoral(),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("Test-Blu-CoralTopToReef10", false, false),
      new TeleopMoveToL4RotateArm(),
      new TeleopEjectCoralBringArmToCruise()
    );
  }
}
