// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimiter;
import frc.robot.Constants.EnableCurrentLimiter;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.GPMConstants.IntakeConstants;
import frc.robot.Constants.GPMConstants.IntakeConstants.IntakeCoralCANRangeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  // NEO motors connected to Spark Max
  private SparkMax intakeMotor;

  private SparkClosedLoopController intakePIDController;
  private RelativeEncoder intakePIDEncoder;

  public static DigitalInput GPMsensors;
  private DigitalInput intakeDownLimitSwitch;

  private CANrange intakeSensor;
  StatusSignal<Distance> distanceToTarget;
  StatusSignal<Boolean> targetVisible;

  public IntakeSubsystem() {

    // Check if need to initialize arm
    if (!EnabledSubsystems.intake) {
      return;
    }

    intakeMotor = new SparkMax(IntakeConstants.INTAKE_ROLLERMOTOR_CAN_ID, MotorType.kBrushless);

    intakePIDController = intakeMotor.getClosedLoopController();

    // Set Arm encoders
    intakePIDEncoder = intakeMotor.getEncoder();

    // Main Motor; should not follow the other motor
    configureIntakeMotor(intakeMotor, intakePIDEncoder, intakePIDController);



    // Initialize the CANRange sensor with the appropriate CAN ID
    intakeSensor = new CANrange(IntakeCoralCANRangeConstants.intakeCANRangeID); // Replace '1' with the actual CAN ID of your sensor

    // Configure the sensor for short-distance detection
    configureCANRange();
    distanceToTarget = intakeSensor.getDistance();
    System.out.println("*** Intake initialized");
  }

    private void configureCANRange() {
    CANrangeConfiguration config = new CANrangeConfiguration();
    config.withProximityParams(new ProximityParamsConfigs()
        .withProximityThreshold(IntakeCoralCANRangeConstants.newProximityThreshold));
    config.withToFParams(new ToFParamsConfigs()
        .withUpdateMode(UpdateModeValue.ShortRangeUserFreq)
        .withUpdateFrequency(IntakeCoralCANRangeConstants.newUpdateFrequency));
    config.withFovParams(new FovParamsConfigs()
        .withFOVRangeX(IntakeCoralCANRangeConstants.intakeFOVRangeX)
        .withFOVRangeY(IntakeCoralCANRangeConstants.intakeFOVRangeY)
        );
    intakeSensor.getConfigurator().apply(config);
  }

  public double getRange() {
    return distanceToTarget.refresh().getValueAsDouble();
  }

  private void configureIntakeMotor(SparkMax motor,  RelativeEncoder encoder, SparkClosedLoopController p) {

    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

    //motor.restoreFactoryDefaults(); //restores the state of the motor to factory defaults
    motor.clearFaults();  //clears a fault that has occurred since the last time the faults were reset
    sparkMaxConfig.inverted(IntakeConstants.INTAKE_ROLLERMOTOR_INVERTED); //sets motor inverted if getArmMotorInverted() returns true

    sparkMaxConfig.idleMode(IdleMode.kBrake); //sets motor into brake mode
    //motor.setIdleMode(IdleMode.kCoast); 

    EncoderConfig encoderConfig = new EncoderConfig();
    encoderConfig.positionConversionFactor(IntakeConstants.POSITION_CONVERSION_FACTOR);  //sets conversion between NEO units to necessary unit for positon
    sparkMaxConfig.apply(encoderConfig);

    motor.setCANTimeout(0); //sets up timeout

    sparkMaxConfig.voltageCompensation(IntakeConstants.nominalVoltage);  //enables voltage compensation for set voltage [12v]
   
    if (EnableCurrentLimiter.intake) {
      sparkMaxConfig.smartCurrentLimit(CurrentLimiter.arm); // sets current limit to 40 amps
    }
    
    sparkMaxConfig.openLoopRampRate(IntakeConstants.rampRate);  // sets the rate to go from 0 to full throttle on open loop
    sparkMaxConfig.closedLoopRampRate(IntakeConstants.rampRate);  // sets the rate to go from 0 to full throttle on open loop


    SignalsConfig signalsConfig = new SignalsConfig();

    // apply signals
    sparkMaxConfig.apply(signalsConfig);

    // ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    // // --- PID Setup
    // // set the PID sensor to motor encoder for hardware PID
    // closedLoopConfig.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // // set arm PID coefficients - LIFT
    // closedLoopConfig.p(IntakePIDConstants.kP);
    // closedLoopConfig.i(IntakePIDConstants.kI);
    // closedLoopConfig.d(IntakePIDConstants.kD);
    // closedLoopConfig.iZone(IntakePIDConstants.Izone);
    // //p.setFF(ArmPIDConstants.kF);
    // // kMaxOutput = 1 ; range is -1, 1
    // closedLoopConfig.outputRange(-IntakePIDConstants.kMaxOutput, IntakePIDConstants.kMaxOutput);

    // // Apply closed loop configuration
    // sparkMaxConfig.apply(closedLoopConfig);

    motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runIntake(double speed) {
    System.out.println("Running Intake with speed: " +  speed);
    intakeMotor.set(speed);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  


  public double getDistanceToTarget() {
    return distanceToTarget.refresh().getValueAsDouble();
  }

  public boolean isTargetVisible() {
    return intakeSensor.getIsDetected().getValue(); 
  }

  public double getOutputCurrent() {
    return intakeMotor.getOutputCurrent();
  }

  public boolean isAlgaeHeld() {
    return getOutputCurrent() >= IntakeConstants.algaeStallCurrent;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
