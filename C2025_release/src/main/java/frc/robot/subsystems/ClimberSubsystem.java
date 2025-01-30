// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.GPMConstants.ClimberConstants;
import frc.robot.Constants.SwerveConstants.Intake;

public class ClimberSubsystem extends SubsystemBase {
   private TalonFX climberMotor;
   private RelativeEncoder climberEncoder;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    // Check if need to initialize arm
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
    var talonFXConfigurator = climberMotor.getConfigurator();
    motorconfigs.Inverted = (Intake.INTAKE_INVERTED ? InvertedValue.CounterClockwise_Positive: InvertedValue.Clockwise_Positive);
    talonFXConfigurator.apply(motorconfigs);
  }

  public void climbUP() {
    climberMotor.set(Constants.GPMConstants.ClimberConstants.climbUpPower);
  }

  public void climbDown() {
    climberMotor.set(Constants.GPMConstants.ClimberConstants.climbDownPower);
  }

  public void climbToSpeed(double speed) {
    climberMotor.set(speed);
  }

  public void stopClimbMotor() {
    climberMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
