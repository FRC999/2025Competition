// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBlu2Coral extends SequentialCommandGroup {
  /** Creates a new AutoBlu2Coral. */
  public AutoBlu2Coral() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand( ()-> RobotContainer.driveSubsystem.setOdometryToIdealPoseFromTrajectory("Blu-BargeToReef11"))
        // just in case - wait 0.1s for the pose to take place
        .andThen(new WaitCommand(0.1))
        .andThen(RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-BargeToReef11", true,false))
        .andThen(new WaitCommand(0.1))
        .andThen(new TeleopMoveToL4RotateArm()) // try placing Coral on L4
        .andThen(new TeleopEjectCoralBringArmToCruiseElevatorDown())//TODO: needs to be looked over, especially if we have to 
                                                                    //add the trajectory which will make the bot go backwards 
                                                                    //before putting the elevator down. 
        .andThen(new WaitCommand(0.1))
        .andThen(RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-Reef11ToCoralTop",false,false))
        .andThen(new WaitCommand(0.1))
        .andThen(new TeleopCoralIntakeSequence())
        .andThen(new WaitCommand(0.1))
        .andThen(RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-CoralTopToReef10", false, false))
        .andThen(new TeleopMoveToL4RotateArm())  // try placing Coral on L4
        .andThen(new TeleopEjectCoralBringArmToCruiseElevatorDown())
    );
  }
}
