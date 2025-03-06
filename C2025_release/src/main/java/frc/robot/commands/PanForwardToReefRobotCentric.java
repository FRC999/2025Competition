// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.GPMConstants.IntakeConstants.PerimeterFinderConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PanForwardToReefRobotCentric extends Command {

  boolean doNotRun = true;

  /** Creates a new PanForwardToReef. */
  public PanForwardToReefRobotCentric() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem, RobotContainer.perimeterFinderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("*** PanForwardToReefTarget command started");
    if (RobotContainer.perimeterFinderSubsystem.isTargetVisibleFront() 
      && RobotContainer.perimeterFinderSubsystem.getDistanceToTargetFront() < PerimeterFinderConstants.minDistance) {
        System.out.println("*** PanForwardToReefTarget drive started");
        doNotRun =  false;
        // Drive forward, slowly
        RobotContainer.driveSubsystem.driveRobotCentric(PerimeterFinderConstants.driveVelocity, 0, 0);
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop Robot
    RobotContainer.driveSubsystem.driveRobotCentric(0, 0, 0);
    System.out.println("*** PanForwardToReefTarget command ended " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return doNotRun || RobotContainer.perimeterFinderSubsystem.getDistanceToTargetFront() < PerimeterFinderConstants.minDistance;
  }
}
