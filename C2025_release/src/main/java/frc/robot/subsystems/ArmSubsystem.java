// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.GPMConstants.ArmConstants;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmHeights;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmPIDConstants;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmPIDConstants.MotionMagicDutyCycleConstants;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmPIDConstants.MotionMagicVoltageConstants;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmPIDConstants.PositionDutyCycleConstants;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmPIDConstants.PositionVoltageConstants;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorHeights;
import frc.robot.Constants.GPMConstants.ElevatorConstants.ElevatorPIDConstants;
import frc.robot.Constants.SwerveConstants.Intake;

import java.time.LocalTime;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
   private TalonFX armMotor; // Kraken
   private final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

   private MotionMagicDutyCycle motMagDutyCycle = new MotionMagicDutyCycle(0); // for MotionMagic Duty Cycle
   private PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0); // for position voltage
   private MotionMagicVoltage motMagVoltage = new MotionMagicVoltage(0); // for MotionMagic Voltage

   private RelativeEncoder armEncoder;
   private double armEncoderZero;
   private CANcoder armCANCoder;


  public ArmSubsystem() {

    // Check if need to initialize arm
    if (!EnabledSubsystems.arm) {
      return;
    }

    armMotor = new TalonFX(ArmConstants.ARM_MOTOR_CAN_ID);
    armCANCoder = new CANcoder(ArmConstants.THROUGHBORE_ENCODER_CAN_ID);

    configureArmMotor();
    configureCANCoder();
    calibrateZeroArmPosition();
  }

  public void configureCANCoder() {
    /* Configure CANcoder */
    var toApply = new CANcoderConfiguration();

    /* User can change the configs if they want, or leave it empty for factory-default */
    armCANCoder.getConfigurator().apply(toApply);

    /* Speed up signals to an appropriate rate */
    BaseStatusSignal.setUpdateFrequencyForAll(100, armCANCoder.getPosition(), armCANCoder.getVelocity());
  }

  public double getAbsoluteCANCoderValue() {
    return armCANCoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getRelativeCANCoderValue() {
    return armCANCoder.getPosition().getValueAsDouble();
  }

  public void calibrateZeroArmPosition() {
    armEncoderZero = (ArmConstants.CANCODER_ABSOLUTE_HORIZONTAL_VALUE - getAbsoluteCANCoderValue()) 
        * ArmConstants.MOTOR_ROTATIONS_PER_CANCODER_ROTATIONS 
        + getMotorEncoder();
  }

  private void configureArmMotor() {

    //intakeMotor.configFactoryDefault();
    armMotor.getConfigurator().apply(new TalonFXConfiguration()); // reset the motor to defaults
    armMotor.setSafetyEnabled(false);

    var motorconfigs = new MotorOutputConfigs();
    motorconfigs.NeutralMode = NeutralModeValue.Brake;
    motorconfigs.Inverted = (ArmConstants.ARM_INVERTED ? InvertedValue.CounterClockwise_Positive: InvertedValue.Clockwise_Positive);
    var talonFXConfigurator = armMotor.getConfigurator();
    TalonFXConfiguration pidConfig = new TalonFXConfiguration();
    configurePositionDutyCycle(pidConfig);
    talonFXConfigurator.apply(motorconfigs);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = talonFXConfigurator.apply(pidConfig);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }
  public void runArm(double speed) {
    armMotor.set(speed);
  }

  public void stopArm() {
    armMotor.setControl(new DutyCycleOut(0));
  }


  private void configurePositionDutyCycle(TalonFXConfiguration config) {
    config.Slot0.kV = PositionDutyCycleConstants.arm_kV;
    config.Slot0.kP = PositionDutyCycleConstants.arm_kP;
    config.Slot0.kI = PositionDutyCycleConstants.arm_kI;
    config.Slot0.kD = PositionDutyCycleConstants.arm_kD;
  }

  private void setPositionDutyCycle(double position){
    armMotor.setControl(new PositionDutyCycle(position));
  }

  private void configurePositionVoltage(TalonFXConfiguration config) {
    config.Slot0.kP = PositionVoltageConstants.arm_kP;
    config.Slot0.kI = PositionVoltageConstants.arm_kI;
    config.Slot0.kD = PositionVoltageConstants.arm_kD;
    
    config.Voltage.withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(-8));
  }

  private void setPositonVoltage(double position){
    armMotor.setControl(positionVoltage.withPosition(position));
  }

  private void configureMotionMagicDutyCycle(TalonFXConfiguration config) {
    //config.Slot0.kS = 0.24; // add 0.24 V to overcome friction
    //config.Slot0.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    //PID on Position
    config.Slot0.kP = MotionMagicDutyCycleConstants.arm_kP;
    config.Slot0.kI = MotionMagicDutyCycleConstants.arm_kI;
    config.Slot0.kD = MotionMagicDutyCycleConstants.arm_kD;

    config.MotionMagic.MotionMagicCruiseVelocity = MotionMagicDutyCycleConstants.MotionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = MotionMagicDutyCycleConstants.motionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = MotionMagicDutyCycleConstants.motionMagicJerk;

    motMagDutyCycle.Slot = MotionMagicDutyCycleConstants.slot;
  }

  private void setMotionMagicDutyCycle(double position){
    armMotor.setControl(motMagDutyCycle.withPosition(position));
  }

  private void configureMotionMagicVoltage(TalonFXConfiguration config) {
    config.Slot0.kS = MotionMagicVoltageConstants.arm_kS; // add 0.24 V to overcome friction
    config.Slot0.kV = MotionMagicVoltageConstants.arm_kV; // apply 12 V for a target velocity of 100 rps
    //PID on Position
    config.Slot0.kP = MotionMagicVoltageConstants.arm_kP;
    config.Slot0.kI = MotionMagicVoltageConstants.arm_kI;
    config.Slot0.kD = MotionMagicVoltageConstants.arm_kD;

    config.MotionMagic.MotionMagicCruiseVelocity = MotionMagicVoltageConstants.MotionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = MotionMagicVoltageConstants.motionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = MotionMagicVoltageConstants.motionMagicJerk;

    motMagVoltage.Slot = MotionMagicVoltageConstants.slot;
  }

  private void setMotionMagicVoltage(double position){
    armMotor.setControl(motMagVoltage.withPosition(position));
  }


  public double getMotorEncoder() {
    return armMotor.getRotorPosition().getValueAsDouble();
  }

  public void setArmPositionWithHeight(ArmHeights height) { 
    setPositionDutyCycle(armEncoderZero + height.getHeight());
  }

  public boolean isAtPosition(ArmHeights position){
    return Math.abs(position.getHeight() - getMotorEncoder())<=ArmPIDConstants.tolerance;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
