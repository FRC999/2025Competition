// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTrajectoryBluBargeToReef11 extends SequentialCommandGroup {
  /** Creates a new AutoTrajectoryBluBargeToReef11. */
  public AutoTrajectoryBluBargeToReef11() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    try {
      addCommands(
        new PrintCommand("Button pressed start"),
        new AutonomousTrajectory2Poses(new Pose2d(4.910, 5.080, new Rotation2d()), new Pose2d(8.270, 6.130, new Rotation2d())),
        new PrintCommand("Button pressed end")
        //new ElevatorToLevelForCoralPlacement(ElevatorHeights.ReefLevelFour, ArmPositions.ReefLevelFour),
        //new IntakeCoralOutCommand(0.2)
      );
    } catch (Exception e) {
      System.out.println("***Error: " + e + "! ***");
    }
  }
}
