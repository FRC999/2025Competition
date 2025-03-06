// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PanForwardOrBackWithLowSpeed extends Command {
  double panVelocity;
  /** Creates a new PanForwardWithLowSpeed. */
  public PanForwardOrBackWithLowSpeed(double pVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("*** PanForwardOrBackWithLowSpeed command started ");

    // Since our bot is not necessarily straight and we need to pan to the side, we need to get X and Y components of that move
    Rotation2d currentYaw = RobotContainer.driveSubsystem.getYawRotation2d();
    double xComponent = Math.cos(currentYaw.getRadians());
    double yComponent = Math.sin(currentYaw.getRadians());

    RobotContainer.driveSubsystem.drive(panVelocity*xComponent, panVelocity*yComponent, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("*** PanToReefTarget command ended " + interrupted);
  }

  // This command should always be interrupted
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
