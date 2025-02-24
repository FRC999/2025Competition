// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoralCommand extends Command {
  /** Creates a new IntakeCommand. */
  int counter = 0;
  double setYVelocity;
  public IntakeCoralCommand(double yVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intakeSubsystem);
    setYVelocity = yVelocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("*** IntakeCoralCommand started ");
    RobotContainer.intakeSubsystem.runIntake(setYVelocity);
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.stopIntake();
    counter = 0;
    System.out.println("*** IntakeCoral command ended " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("****t: " + RobotContainer.intakeSubsystem.isTargetVisible() + " d: "+ RobotContainer.intakeSubsystem.getDistanceToTarget() + " c " + counter);

    if (RobotContainer.intakeSubsystem.isTargetVisible() &&
       counter == 0 && RobotContainer.intakeSubsystem.getDistanceToTarget() < 0.2) {
      counter++;
    }
    if ( counter != 0) {
      counter++;
    }
    return counter > 4;
  }
}
