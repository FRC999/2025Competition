// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.LocalTime;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CalibrateIntake1 extends InstantCommand {
  public CalibrateIntake1() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("**** Calibrating Elevator ...");
    System.out.println("Start time: " + LocalTime.now());
    double intakePower = RobotContainer.driveStick1.getRawAxis(OIConstants.CALIBRATION_JOYSTICK_SLIDER_AXLE);
    RobotContainer.intakeSubsystem.runIntake(intakePower);
    System.out.println("End time: " + LocalTime.now());
  }


}
