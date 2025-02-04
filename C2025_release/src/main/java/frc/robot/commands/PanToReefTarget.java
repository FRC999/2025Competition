// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.GPMConstants.IntakeConstants.ReefFinderConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PanToReefTarget extends Command {
  /** Creates a new PanToReefTarget. */

  double setYVelocity;
  public PanToReefTarget(double yVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    setYVelocity = yVelocity;
    addRequirements(RobotContainer.driveSubsystem, RobotContainer.reefFinderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("*** PanToReefTarget command started ");
    RobotContainer.driveSubsystem.drive(0, setYVelocity, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("*** PanToReefTarget command ended " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.reefFinderSubsystem.isTargetVisible() && 
      RobotContainer.reefFinderSubsystem.getDistanceToTarget()<=ReefFinderConstants.minDistanceToTarget;
  }
}
