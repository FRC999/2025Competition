// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoralToPan extends Command {
  double setYVelocity;
  /** Creates a new IntakeCoralToPan. */
  public IntakeCoralToPan(double yVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intakeSubsystem);
    setYVelocity = yVelocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("*** IntakeCoralCommand started ");
    RobotContainer.intakeSubsystem.runIntake(setYVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("*** Coral In Pan " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.intakeSubsystem.isPreIntakeTargetVisible() || RobotContainer.intakeSubsystem.isTargetVisible() || 
      RobotContainer.intakeSubsystem.isPostIntakeTargetVisible();
  }
}
