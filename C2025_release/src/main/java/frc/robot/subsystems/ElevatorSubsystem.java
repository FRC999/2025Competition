// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants.Intake;

public class ElevatorSubsystem extends SubsystemBase { //TODO: Need to updated 
  /** Creates a new ElevatorSubsystem. */

  private TalonFX elevatorMotorLeader;
  private TalonFX elevatorMotorFollower;
  public ElevatorSubsystem() {
    initializeElevator();
  }

  public void initializeElevator() {
    elevatorMotorLeader =  new TalonFX(Constants.GPMConstants.ElevatorConstants.ElevatorMotorConstantsEnum.LEADERMOTOR.getElevatorMotorID());
    elevatorMotorFollower =  new TalonFX(Constants.GPMConstants.ElevatorConstants.ElevatorMotorConstantsEnum.LEADERMOTOR.getElevatorMotorID());
    
    elevatorMotorFollower.setControl(new Follower(elevatorMotorLeader.getDeviceID(), false));
    elevatorMotorLeader.getConfigurator().apply(new TalonFXConfiguration()); // reset the motor to defaults
    var talonFXConfigurator = elevatorMotorLeader.getConfigurator();
    
    var motorconfigs = new MotorOutputConfigs();
    motorconfigs.Inverted = (Intake.INTAKE_INVERTED ? InvertedValue.CounterClockwise_Positive: InvertedValue.Clockwise_Positive);
    talonFXConfigurator.apply(motorconfigs);
    elevatorMotorLeader.setSafetyEnabled(false);


    // class member variable
    final VelocityVoltage m_velocity = new VelocityVoltage(0);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // robot init, set slot 0 gains
    Slot0Configs slot0Configs = config.Slot0;
    slot0Configs.kP = Constants.GPMConstants.ElevatorConstants.elevator_kP;
    slot0Configs.kI = Constants.GPMConstants.ElevatorConstants.elevator_kI;
    slot0Configs.kD = Constants.GPMConstants.ElevatorConstants.elevator_kD;
    slot0Configs.kG = Constants.GPMConstants.ElevatorConstants.elevator_kG;
    talonFXConfigurator.apply(slot0Configs, 0.050);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = elevatorMotorLeader.getConfigurator().apply(config);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }
  

  public double getElevatorEncoder() {
    return elevatorMotorLeader.getRotorPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
