// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.GPMConstants.ElevatorConstants;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorHeights;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorPIDConstants;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorPIDConstants.MotionMagicDutyCycleConstants;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorPIDConstants.MotionMagicVoltageConstants;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorPIDConstants.PositionDutyCycleConstants;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorPIDConstants.PositionVoltageConstants;

public class ElevatorSubsystem extends SubsystemBase { //TODO: Need to updated 
  /** Creates a new ElevatorSubsystem. */

  private TalonFX elevatorMotorLeader;
  private TalonFX elevatorMotorFollower;
  private DigitalInput limitSwitch;
  private double zeroPosition; //Relative Encoder Setting for zero position of elevator

  
  MotionMagicDutyCycle motMagDutyCycle = new MotionMagicDutyCycle(0);
  MotionMagicVoltage motMagVoltage = new MotionMagicVoltage(0);
  PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);

  private double minHeight = ElevatorHeights.ReefLevelTwo.getHeight();

  public ElevatorSubsystem() {

    // Check if need to initialize arm
    if (!EnabledSubsystems.elevator) {
      return;
    }

    elevatorMotorLeader =  new TalonFX(ElevatorConstants.ELEVATOR_LEADERMOTOR_CAN_ID);
    elevatorMotorFollower =  new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWERMOTOR_CAN_ID);

    configureElevatorMotors();
  }


  public void configureElevatorMotors() {
    elevatorMotorLeader.getConfigurator().apply(new TalonFXConfiguration()); // reset the motor to defaults
    elevatorMotorFollower.getConfigurator().apply(new TalonFXConfiguration());

    elevatorMotorFollower.setSafetyEnabled(false);
    elevatorMotorLeader.setSafetyEnabled(false);
    
    var talonFXConfiguratorLeader = elevatorMotorLeader.getConfigurator();
    var talonFXConfiguratorFollower = elevatorMotorFollower.getConfigurator(); 
    
    var motorconfigs = new MotorOutputConfigs();
    motorconfigs.NeutralMode = NeutralModeValue.Brake; 

    motorconfigs.Inverted = (ElevatorConstants.ELEVATOR_MOTOR_INVERTED ? InvertedValue.CounterClockwise_Positive: InvertedValue.Clockwise_Positive);
    
    TalonFXConfiguration pidConfig = new TalonFXConfiguration().withMotorOutput(motorconfigs);
    configureMotionMagicDutyCycle(pidConfig);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = talonFXConfiguratorLeader.apply(pidConfig);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    for (int i = 0; i < 5; ++i) {
      status = talonFXConfiguratorFollower.apply(pidConfig);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    if (ElevatorConstants.IS_LIMIT_SWITCH_PRESENT) {
      try {
        limitSwitch = new DigitalInput(ElevatorConstants.ELEVATOR_DOWN_LIMIT_SWITCH_DIO_PORT_NUMBER);
        System.out.println("*** Elvator Down Limit Switch initialized");
      } catch (Exception e) {
        System.out.println("Unable to get elevator down limit switch value");
      }
    }

    elevatorMotorFollower.setControl(new Follower(ElevatorConstants.ELEVATOR_LEADERMOTOR_CAN_ID, false));

  }

  private void configurePositionDutyCycle(TalonFXConfiguration config) {
    config.Slot0.kV = PositionDutyCycleConstants.elevator_kV;
    config.Slot0.kP = PositionDutyCycleConstants.elevator_kP;
    config.Slot0.kI = PositionDutyCycleConstants.elevator_kI;
    config.Slot0.kD = PositionDutyCycleConstants.elevator_kD;
  }

  private void setPositionDutyCycle(double position){
    elevatorMotorLeader.setControl(new PositionDutyCycle(position));
  }

  private void configurePositionVoltage(TalonFXConfiguration config) {
    config.Slot0.kP = PositionVoltageConstants.elevator_kP;
    config.Slot0.kI = PositionVoltageConstants.elevator_kI;
    config.Slot0.kD = PositionVoltageConstants.elevator_kD;
    
    config.Voltage.withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(-8));
  }

  private void setPositonVoltage(double position){
    elevatorMotorLeader.setControl(positionVoltage.withPosition(position));
  }

  private void configureMotionMagicDutyCycle(TalonFXConfiguration config) {
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

  private void setMotionMagicDutyCycle(double position){
    elevatorMotorLeader.setControl(motMagDutyCycle.withPosition(position));
  }

  private void configureMotionMagicVoltage(TalonFXConfiguration config) {
    config.Slot0.kS = MotionMagicVoltageConstants.elevator_kS; // add 0.24 V to overcome friction
    config.Slot0.kV = MotionMagicVoltageConstants.elevator_kV; // apply 12 V for a target velocity of 100 rps
    //PID on Position
    config.Slot0.kP = MotionMagicVoltageConstants.elevator_kP;
    config.Slot0.kI = MotionMagicVoltageConstants.elevator_kI;
    config.Slot0.kD = MotionMagicVoltageConstants.elevator_kD;

    config.MotionMagic.MotionMagicCruiseVelocity = MotionMagicVoltageConstants.MotionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = MotionMagicVoltageConstants.motionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = MotionMagicVoltageConstants.motionMagicJerk;

    motMagVoltage.Slot = MotionMagicVoltageConstants.slot;
  }

  private void setMotionMagicVoltage(double position){
    elevatorMotorLeader.setControl(motMagVoltage.withPosition(position));
  }

  public double getHeightAdjustmentCoefficient() {
    double currentHeight = getElevatorHeight();
    if(currentHeight < minHeight) {
      return 1;
    } else {
      return 0.25;
    }
  }
  
  public double getMotorEncoder() {
    return elevatorMotorLeader.getRotorPosition().getValueAsDouble();
  }

  public double getElevatorHeight() {
    return elevatorMotorLeader.getRotorPosition().getValueAsDouble();
  }


  public boolean isLimitSwitchPressed() {
    return (limitSwitch.get());
  }

  private void setZeroPosition() {
    zeroPosition = getMotorEncoder();
  }

  public void setElevatorPositionWithHeight(ElevatorHeights height) { 
    setMotionMagicDutyCycle(zeroPosition+height.getHeight());
  }

  public void runElevatorWithDutyCycle(double speed) {
    elevatorMotorLeader.set(speed);
  }

  public void runElevatorWithVoltage(double input) {
    elevatorMotorLeader.setVoltage(input*12);
  }

  public void stopElevator() {
    elevatorMotorLeader.setControl(new DutyCycleOut(0));
    //elevatorMotorFollower.setControl(new DutyCycleOut(0));
  }

  public void stopElevatorAndHold() {
    elevatorMotorLeader.setControl(new DutyCycleOut(ElevatorConstants.holdingDutyCycleConstant));
  }

  public boolean isAtHeight(ElevatorHeights height){
    return Math.abs(height.getHeight() - getMotorEncoder())<=ElevatorPIDConstants.tolerance;
  }

  // Test if elevator is down
  public boolean isDown(){
    return getElevatorHeight() <= ElevatorHeights.ReefLevelOne.getHeight() ;
  }

  public double elevatorHeightChassisSpeedAdjustmentCoefficient() { //TODO: change not to convert into meters 
    double coefficient = 0.0;

    if (getElevatorHeight() <= ElevatorHeights.MinimumChassisSpeedReductionHeight.getHeight()) {
      coefficient = 1.0;
    } else {
      coefficient = 1 - ((getElevatorHeight() - ElevatorHeights.MinimumChassisSpeedReductionHeight.getHeight()) 
          / (ElevatorHeights.MaxHeight.getHeight() - ElevatorHeights.MinimumChassisSpeedReductionHeight.getHeight()))
          * 0.5;
    }

    return coefficient;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
