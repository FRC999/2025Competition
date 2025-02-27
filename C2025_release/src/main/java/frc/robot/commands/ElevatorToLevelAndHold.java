// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorHeights;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToLevelAndHold extends Command {

  private ElevatorHeights setHeight;
  /** Creates a new ElevatorToLevelAndHold. */
  public ElevatorToLevelAndHold(ElevatorHeights height) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.elevatorSubsystem);
    setHeight = height;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Elevator going to height: " + setHeight);
    RobotContainer.elevatorSubsystem.setElevatorPositionWithHeight(setHeight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Elevator at height: " + setHeight);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.elevatorSubsystem.isAtHeight(setHeight);
  }
}
