// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.GPMConstants.IntakeConstants;
import frc.robot.Constants.GPMConstants.PanConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopRunPanMotorWithSpeed extends Command {
  /** Creates a new IntakeAlgaeCommand. */
  public TeleopRunPanMotorWithSpeed() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.panSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("*** PanMotorCommand started ");
    RobotContainer.panSubsystem.runPanMotor(PanConstants.panSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.panSubsystem.runPanMotor(0.0);
    System.out.println("*** PanMotor finished ");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.panSubsystem.isPanHeld();
  }
}
