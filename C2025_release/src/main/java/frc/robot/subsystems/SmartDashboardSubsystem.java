// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class SmartDashboardSubsystem extends SubsystemBase {
  /** Creates a new SmartDashboardSubsystem. */
  public SmartDashboardSubsystem() {}

  public void updateOdometryTelemetry() {
    for (int i =0; i<4; i++){
      SmartDashboard.putNumber("S"+i+" Drive Encoder", RobotContainer.driveSubsystem.getDriveEncoder(i));
      SmartDashboard.putNumber("S"+i+" Drive Velocity", RobotContainer.driveSubsystem.getDriveVelocity(i));
      SmartDashboard.putNumber("S"+i+" Drive EncoderSI", RobotContainer.driveSubsystem.getDriveEncoderSI(i));
      SmartDashboard.putNumber("S"+i+" Drive VelocitySI", RobotContainer.driveSubsystem.getDriveVelocitySI(i));
      SmartDashboard.putNumber("S"+i+" CANCoder Absolute SI", RobotContainer.driveSubsystem.getCancoderAbsoluteSI(i));
      SmartDashboard.putNumber("S"+i+" CANCoder Relative SI", RobotContainer.driveSubsystem.getCancoderRelativeSI(i));
   }
  }

  public void updateChassisTelemetry() {
    SmartDashboard.putString("Chassis Pose", RobotContainer.driveSubsystem.getPose().toString());
    SmartDashboard.putString("Chassis Velocity", RobotContainer.driveSubsystem.getState().Speeds.toString());
  }
  
  public void updateIMUTelemetry() {
    SmartDashboard.putNumber("IMU Yaw", RobotContainer.driveSubsystem.getYaw());
    SmartDashboard.putNumber("Rotation2D: ", RobotContainer.driveSubsystem.getRotation2d().getRadians()); 
    SmartDashboard.putNumber("Imu Speed: ", RobotContainer.driveSubsystem.getPigeon2Speed());
  }

  public void updateArmTelemetry() {
    SmartDashboard.putNumber("Arm ThroughBore Encoder", RobotContainer.armSubsystem.getArmSparkThroughboreEncoderPosition());
    SmartDashboard.putNumber("Arm Motor Encoder Value", RobotContainer.armSubsystem.getKrakenMotorEncoder());
  }

  public void updateElevatorTelemetry() {
    SmartDashboard.putNumber("Elevator Motor Encoder Value", RobotContainer.elevatorSubsystem.getMotorEncoder());
  }

  public void updateClimberTelemetry() {
    SmartDashboard.putNumber("Climber Motor Encoder Value", RobotContainer.climberSubsystem.getMotorEncoder());
  }

  public void updateIntakeTelemetry() {
    SmartDashboard.putNumber("IntakeCANrange Distance: ", RobotContainer.intakeSubsystem.getRange());
    SmartDashboard.putBoolean("IntakeCANRangeTargetVisible: ", RobotContainer.intakeSubsystem.isTargetVisible());
    SmartDashboard.putNumber("Intake Output Current: ", RobotContainer.intakeSubsystem.getOutputCurrent());
  }

  public void updateReefFinderTelemetry() {
    SmartDashboard.putBoolean("Reef Target Visibility: ", RobotContainer.reefFinderSubsystem.isTargetVisible());
    SmartDashboard.putNumber("Distance to Reef Target: ", RobotContainer.reefFinderSubsystem.getDistanceToTarget());
  }

  public void updateAllDisplays(){
    if (DebugTelemetrySubsystems.imu) {
      updateIMUTelemetry();
    }

    if (DebugTelemetrySubsystems.chasis) {
      updateChassisTelemetry();
    }

    if (DebugTelemetrySubsystems.odometry) {
      updateOdometryTelemetry();
    }

    if (DebugTelemetrySubsystems.arm) {
      updateArmTelemetry();
    }

    if (DebugTelemetrySubsystems.elevator) {
      updateElevatorTelemetry();
    }

    if (DebugTelemetrySubsystems.climber) {
      updateClimberTelemetry();
    }
    if(DebugTelemetrySubsystems.intake) {
      updateIntakeTelemetry();
    }

    if(DebugTelemetrySubsystems.reef) {
      updateReefFinderTelemetry();
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateAllDisplays();
  }
}
