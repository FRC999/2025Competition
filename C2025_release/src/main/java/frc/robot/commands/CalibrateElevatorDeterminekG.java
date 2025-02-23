// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.LocalTime;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CalibrateElevatorDeterminekG extends Command {
  /** Creates a new CalibrateElevatorDetermineKg. */
  public CalibrateElevatorDeterminekG() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("**** Calibrating Elevator ...");
    System.out.println("Start time: " + LocalTime.now());
    double elevatorPower = RobotContainer.driveStick1.getRawAxis(OIConstants.CALIBRATION_JOYSTICK_SLIDER_AXLE);
    RobotContainer.elevatorSubsystem.runElevatorWithDutyCycle(elevatorPower*0.1);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elevatorPower = RobotContainer.driveStick1.getRawAxis(OIConstants.CALIBRATION_JOYSTICK_SLIDER_AXLE);
    RobotContainer.elevatorSubsystem.runElevatorWithDutyCycle(elevatorPower*0.4);
    SmartDashboard.putNumber("Calibration - Elevator Power", elevatorPower*0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.elevatorSubsystem.stopElevatorAndHold();
    System.out.println("End time: " + LocalTime.now());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
