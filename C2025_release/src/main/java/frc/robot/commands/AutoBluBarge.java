// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorHeights;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBluBarge extends SequentialCommandGroup {
  /** Creates a new TestBluBarge. */
  public AutoBluBarge() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-BargeToCenterReef", true, false),
      new TeleopMoveToL4RotateArm(),
      new TeleopEjectCoralBringArmToCruise(),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-MidGoBack", false, false),
      new AutoAlgaeIntakeArmPosition(),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-MidToAlgae", false, false)
        .alongWith(
            new IntakeAlgaeRollerInAndHold()
          ),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-Reef6ToBargeNet", false, false),
      new WaitCommand(0.1),
      new AlgaeToBarge(),
      new ElevatorAllTheWayDown(),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-BargeNetToCoralStationPosition", false, false)
    );
  }
}
