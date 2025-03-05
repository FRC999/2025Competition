// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.GPMConstants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
   private TalonFX climberMotor;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    // Check if need to initialize climber
    if (!EnabledSubsystems.climber) {
      return;
    }

    climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_CAN_ID);

    configureClimberMotor();
  }

  public void configureClimberMotor(){
    climberMotor.getConfigurator().apply(new TalonFXConfiguration()); // reset the motor to defaults
    climberMotor.setSafetyEnabled(false);
    var motorconfigs = new MotorOutputConfigs();
    motorconfigs.NeutralMode = NeutralModeValue.Brake; 
    motorconfigs.Inverted = (ClimberConstants.climberInverted ? InvertedValue.CounterClockwise_Positive: InvertedValue.Clockwise_Positive);

    var talonFXConfigurator = climberMotor.getConfigurator();
    TalonFXConfiguration pidConfig = new TalonFXConfiguration().withMotorOutput(motorconfigs);

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

  public double getMotorEncoder() {
    return climberMotor.getRotorPosition().getValueAsDouble();
  }

  public void climbUP() {
    climberMotor.set(Constants.GPMConstants.ClimberConstants.climbUpPower);
  }

  public void climbDown() {
    climberMotor.set(Constants.GPMConstants.ClimberConstants.climbDownPower);
  }

  public void climbToSpeed(double speed) {
    climberMotor.setControl(new DutyCycleOut(speed));
  }

  public void stopClimbMotor() {
    climberMotor.setControl(new DutyCycleOut(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
