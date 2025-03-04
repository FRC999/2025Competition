// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.GPMConstants.ArmConstants;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmPositions;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmPIDConstants;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmPIDConstants.MotionMagicDutyCycleConstants;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmPIDConstants.MotionMagicVoltageConstants;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmPIDConstants.PositionDutyCycleConstants;
import frc.robot.Constants.GPMConstants.ArmConstants.ArmPIDConstants.PositionVoltageConstants;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
   private TalonFX armMotorKraken; // Kraken
   private final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

   private MotionMagicDutyCycle motMagDutyCycle = new MotionMagicDutyCycle(0); // for MotionMagic Duty Cycle
   private PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0); // for position voltage
   private MotionMagicVoltage motMagVoltage = new MotionMagicVoltage(0); // for MotionMagic Voltage

   private double armKrakenEncoderZero = 0;

   private SparkMax sparkMax;
  private SparkAbsoluteEncoder armSparkThroughboreEncoder;


  public ArmSubsystem() {

    // Check if need to initialize arm
    if (!EnabledSubsystems.arm) {
      return;
    }

    armMotorKraken = new TalonFX(ArmConstants.ARM_MOTOR_CAN_ID);

    // Initialize the Spark MAX (motor type doesn't matter if no motor is connected)
    sparkMax = new SparkMax(ArmConstants.THROUGHBORE_ENCODER_CAN_ID, MotorType.kBrushless);

    // Initialize the through-bore absolute encoder
    armSparkThroughboreEncoder = sparkMax.getAbsoluteEncoder();

    configureArmMotor();
    configureThroughboreEncoder();
    getArmSparkThroughboreEncoderPosition();
    getArmSparkThroughboreEncoderPosition();
    getArmSparkThroughboreEncoderPosition();
    getArmSparkThroughboreEncoderPosition();
    //calibrateZeroArmPosition();
  }

  private void configureThroughboreEncoder() {
    SignalsConfig signalsConfig = new SignalsConfig();

    // kstatus0
    signalsConfig.faultsPeriodMs(100);
    signalsConfig.appliedOutputPeriodMs(100);
    signalsConfig.outputCurrentPeriodMs(100);

    // kstatus1
    signalsConfig.motorTemperaturePeriodMs(250);
    //signalsConfig.primaryEncoderVelocityPeriodMs(250);
    signalsConfig.absoluteEncoderPositionPeriodMs(10);


    AbsoluteEncoderConfig config = new AbsoluteEncoderConfig();
    config.zeroOffset(0);
    SparkMaxConfig througboreSparkConfig = new SparkMaxConfig();
    througboreSparkConfig.apply(signalsConfig);
    througboreSparkConfig.apply(config);
    REVLibError status = null;
    for (int i = 0; i < 5; ++i) {
      status = sparkMax.configure(througboreSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      if (status == REVLibError.kOk)
        break;
    }
    if ( status != null && status != REVLibError.kOk) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
    
  }


  public double getArmSparkThroughboreEncoderPosition() {
    return armSparkThroughboreEncoder.getPosition(); // Get the current position
}
  // public void calibrateZeroArmPosition() {
  //   armEncoderZero = (ArmConstants.CANCODER_ABSOLUTE_HORIZONTAL_VALUE - getAbsoluteCANCoderValue()) 
  //       * ArmConstants.MOTOR_ROTATIONS_PER_CANCODER_ROTATIONS 
  //       + getMotorEncoder();
  // }

  private void configureArmMotor() {

    //intakeMotor.configFactoryDefault();
    armMotorKraken.getConfigurator().apply(new TalonFXConfiguration()); // reset the motor to defaults
    armMotorKraken.setSafetyEnabled(false);


    var motorconfigs = new MotorOutputConfigs();
    motorconfigs.NeutralMode = NeutralModeValue.Brake;
    motorconfigs.Inverted = (ArmConstants.ARM_INVERTED ? InvertedValue.CounterClockwise_Positive: InvertedValue.Clockwise_Positive);
    var talonFXConfigurator = armMotorKraken.getConfigurator();
    //talonFXConfigurator.apply(motorconfigs);

    TalonFXConfiguration pidConfig = new TalonFXConfiguration().withMotorOutput(motorconfigs);
    //configurePositionDutyCycle(pidConfig);
    configureMotionMagicDutyCycle(pidConfig);

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
    armMotorKraken.set(speed);
  }

  public void stopArm() {
    armMotorKraken.setControl(new DutyCycleOut(0));
  }


  private void configurePositionDutyCycle(TalonFXConfiguration config) {
    config.Slot0.kV = PositionDutyCycleConstants.arm_kV;
    config.Slot0.kP = PositionDutyCycleConstants.arm_kP;
    config.Slot0.kI = PositionDutyCycleConstants.arm_kI;
    config.Slot0.kD = PositionDutyCycleConstants.arm_kD;
  }

  private void setPositionDutyCycle(double position){
    armMotorKraken.setControl(new PositionDutyCycle(position));
  }

  private void configurePositionVoltage(TalonFXConfiguration config) {
    config.Slot0.kP = PositionVoltageConstants.arm_kP;
    config.Slot0.kI = PositionVoltageConstants.arm_kI;
    config.Slot0.kD = PositionVoltageConstants.arm_kD;
    
    config.Voltage.withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(-8));
  }

  private void setPositonVoltage(double position){
    armMotorKraken.setControl(positionVoltage.withPosition(position));
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

  public void setMotionMagicDutyCycle(double position){
    armMotorKraken.setControl(motMagDutyCycle.withPosition(position));
    System.out.println("***Pos: " + position);
  }

  public double getArmKrakenEncoderZeroPosition() {
    return armKrakenEncoderZero;
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
    armMotorKraken.setControl(motMagVoltage.withPosition(position));
  }


  public double getKrakenMotorEncoder() {
    return armMotorKraken.getRotorPosition().getValueAsDouble();
  }

  public void setArmPositionWithAngle(ArmPositions angle) { 
    setMotionMagicDutyCycle(armKrakenEncoderZero + angle.getPosition());
  }

  public boolean isAtPosition(ArmPositions position){
    // System.out.println("ME: " + getKrakenMotorEncoder() + " P: " + position.getPosition());
    return Math.abs(position.getPosition() - getKrakenMotorEncoder())<=ArmPIDConstants.tolerance;
  }

  public void calibrateZeroArmPosition() {

    System.out.println("*** Calibrating Arm ...");
    double krakenEncoder = getKrakenMotorEncoder();
    double throughboreEncoder = getArmSparkThroughboreEncoderPosition();
    if(throughboreEncoder > ArmConstants.THROUGHBORE_ENCODER_BEYOND_ALL_THE_WAY_BACK && throughboreEncoder<=1){
      System.out.println("== Throughbore roll over 0");
      throughboreEncoder = throughboreEncoder - 1;
    }
    armKrakenEncoderZero = krakenEncoder - 
      ((ArmConstants.THROUGHBORE_ENCODER_ABSOLUTE_ZERO_VALUE - throughboreEncoder)
      * ArmConstants.MOTOR_ROTATIONS_PER_THROUGHBORE_ROTATIONS);
    System.out.println("EP1: " + throughboreEncoder);
    System.out.println("EP2: " + ((ArmConstants.THROUGHBORE_ENCODER_ABSOLUTE_ZERO_VALUE - throughboreEncoder)
    * ArmConstants.MOTOR_ROTATIONS_PER_THROUGHBORE_ROTATIONS));
    System.out.println("EP3: " + krakenEncoder);
    System.out.println("armKrakenZeroEncoder: " + armKrakenEncoderZero);
  }

  public double armKrakenEncoderRelative() {
    return getKrakenMotorEncoder() - armKrakenEncoderZero;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
