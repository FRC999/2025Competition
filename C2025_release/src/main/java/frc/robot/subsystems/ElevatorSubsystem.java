// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GPMConstants.ElevatorConstants;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorPIDConstants.MotionMagicDutyCycleConstants;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorPIDConstants.MotionMagicVoltageConstants;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorPIDConstants.PositionDutyCycleConstants;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorPIDConstants.PositionVoltageConstants;
import frc.robot.Constants.SwerveConstants.Intake;

public class ElevatorSubsystem extends SubsystemBase { //TODO: Need to updated 
  /** Creates a new ElevatorSubsystem. */

  private TalonFX elevatorMotorLeader;
  private TalonFX elevatorMotorFollower;
  private DigitalInput limitSwitch;
  private double zeroPosition; //Relative Encoder Setting for zero position of elevator
  public ElevatorSubsystem() {
    initializeElevator();
    initializeLimitSwitch();
    setPositionDutyCycle();
    setPositionVoltage();
    setMotionMagicDutyCycle();
    setMotionMagicVoltage();
  }

  public void initializeLimitSwitch() {
    if (ElevatorConstants.elevator_Limit_Switch_isPresent) {
      try {
        limitSwitch = new DigitalInput(ElevatorConstants.elevator_Limit_Switch_port);
        System.out.println("*** Intake Down Limit Switch initialized");
      } catch (Exception e) {
        System.out.println("Unable to get intake down limit switch value");
      }
    }
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

  }

  private void setPositionDutyCycle() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = PositionDutyCycleConstants.elevator_kV;
    slot0Configs.kP = PositionDutyCycleConstants.elevator_kP;
    slot0Configs.kI = PositionDutyCycleConstants.elevator_kI;
    slot0Configs.kD = PositionDutyCycleConstants.elevator_kD;
    elevatorMotorLeader.getConfigurator().apply(slot0Configs, 0.050);
  }

  private void setPositionVoltage() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = PositionVoltageConstants.elevator_kP;
    config.Slot0.kI = PositionVoltageConstants.elevator_kI;
    config.Slot0.kD = PositionVoltageConstants.elevator_kD;
    
    config.Voltage.withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(-8));
    
    PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = elevatorMotorLeader.getConfigurator().apply(config);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  private void setMotionMagicDutyCycle() {
    MotionMagicDutyCycle motMagDutyCycle = new MotionMagicDutyCycle(0);
    TalonFXConfiguration config = new TalonFXConfiguration();
    //config.Slot0.kS = 0.24; // add 0.24 V to overcome friction
    //config.Slot0.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    //PID on Position
    config.Slot0.kP = MotionMagicDutyCycleConstants.elevator_kP;
    config.Slot0.kI = MotionMagicDutyCycleConstants.elevator_kI;
    config.Slot0.kD = MotionMagicDutyCycleConstants.elevator_kD;

    config.MotionMagic.MotionMagicCruiseVelocity = MotionMagicDutyCycleConstants.MotionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = MotionMagicDutyCycleConstants.motionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = MotionMagicDutyCycleConstants.motionMagicJerk;

    elevatorMotorLeader.getConfigurator().apply(config, 0.050);

    motMagDutyCycle.Slot = MotionMagicDutyCycleConstants.slot;
  }

  private void setMotionMagicVoltage() {
    MotionMagicVoltage motMagVoltage = new MotionMagicVoltage(0);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kS = MotionMagicVoltageConstants.elevator_kS; // add 0.24 V to overcome friction
    config.Slot0.kV = MotionMagicVoltageConstants.elevator_kV; // apply 12 V for a target velocity of 100 rps
    //PID on Position
    config.Slot0.kP = MotionMagicVoltageConstants.elevator_kP;
    config.Slot0.kI = MotionMagicVoltageConstants.elevator_kI;
    config.Slot0.kD = MotionMagicVoltageConstants.elevator_kD;

    config.MotionMagic.MotionMagicCruiseVelocity = MotionMagicVoltageConstants.MotionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = MotionMagicVoltageConstants.motionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = MotionMagicVoltageConstants.motionMagicJerk;

    elevatorMotorLeader.getConfigurator().apply(config, 0.050);

    motMagVoltage.Slot = MotionMagicVoltageConstants.slot;
  }
  
  public double getElevatorEncoder() {
    return elevatorMotorLeader.getRotorPosition().getValueAsDouble();
  }


  public boolean isLimitSwitchPressed() {
      return limitSwitch.get();
  }

  private void setZeroPosition() {
    zeroPosition = getElevatorEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
