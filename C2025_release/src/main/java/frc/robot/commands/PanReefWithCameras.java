// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.lib.VisionHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PanReefWithCameras extends Command {

  Pose2d targetPose;
  double panVelocity;
  int counter = 0;

  /** Creates a new PanReefWithCameras. */
  public PanReefWithCameras(Pose2d tpose, double pVelocity) {
    addRequirements(RobotContainer.driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    targetPose = tpose;
    panVelocity = pVelocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("*** PanReef with Camera command started ");
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
    System.out.println("*** PanReef with Camera command ended " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d robotLocation = RobotContainer.llVisionSubsystem.getBestPoseAllCameras();

    double panDistance = VisionHelpers.distanceFromPoseLineY(targetPose, robotLocation);

    // TODO: this is a test for calibration, so remove if not needed
    SmartDashboard.putNumber("PanDistance", panDistance);

    return Math.abs( panDistance ) < AutoConstants.panReefTolerance;
  }
}
