// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.GPMConstants.IntakeConstants.ReefFinderConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PanLeftRightToReefTargetRobotCentric extends Command {
  /** Creates a new PanToReefTarget. */

  double panVelocity;
  int counter = 0;
  public PanLeftRightToReefTargetRobotCentric(double pVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    panVelocity = pVelocity;
    addRequirements(RobotContainer.driveSubsystem, RobotContainer.reefFinderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("*** PanLeftRightToReefTarget command started ");

    RobotContainer.driveSubsystem.driveRobotCentric(0, panVelocity, 0);
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveSubsystem.stopRobot();
    System.out.println("*** PanLeftRightToReefTarget command ended " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println("****t: " + RobotContainer.reefFinderSubsystem.isTargetVisible() + " d: "+ RobotContainer.reefFinderSubsystem.getDistanceToTarget() + " c " + counter);
    if (RobotContainer.reefFinderSubsystem.isTargetVisible() && 
      (RobotContainer.reefFinderSubsystem.getDistanceToTarget()<=ReefFinderConstants.maxDistanceToTarget 
      && RobotContainer.reefFinderSubsystem.getDistanceToTarget()>=ReefFinderConstants.minDistanceToTarget )
      && counter == 0) {
      RobotContainer.driveSubsystem.drive(0, -panVelocity, 0);
      counter++;
    }
    if ( counter != 0) {
      counter++;
    }
    return counter > 3;
  }
}
