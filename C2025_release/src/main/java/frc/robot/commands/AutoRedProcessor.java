// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRedProcessor extends SequentialCommandGroup {
  /** Creates a new TestRedCage. */
  public AutoRedProcessor() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-ProcessortoReef4", false, true)
        .alongWith(
          new WaitCommand(0.5).andThen(
            new TeleopMoveToL4RotateArm()
          )
        ),
      new TeleopEjectCoralBringArmToCruise(),
      new PrintCommand("====intake started"),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-Reef4toCoralBottom", false, true)
        .alongWith(
          new ElevatorAllTheWayDown().andThen(new AutoIntakeSequenceCoral())
        ),
        new PrintCommand("=====Intake Stopped"),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-CoralBottomToReef6Left", false, true)
        .alongWith(
          new WaitCommand(0.5).andThen(
            new TeleopMoveToL4RotateArm()
          )
        ),
      new TeleopEjectCoralBringArmToCruise(),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-Reef6LeftToCoralBottom", false, true)
        .alongWith(
          new ElevatorAllTheWayDown().andThen(  new AutoIntakeSequenceCoral())
        ),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-CoralBottomToReef6Right", false, true)
        .alongWith(
            new WaitCommand(0.5).andThen(
              new TeleopMoveToL4RotateArm()
            )
        ),
      new TeleopEjectCoralBringArmToCruise()
    );
  }
}
