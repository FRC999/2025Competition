// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.GPMConstants.IntakeConstants.CanRangeConstants;
import frc.robot.Constants.GPMConstants.IntakeConstants.ReefFinderConstants;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.TimedRobot;

public class ReefFinderSubsystem extends SubsystemBase {

    private CANrange rangeSensor;
    StatusSignal<Distance> distance;
    StatusSignal<Boolean> targetVisible;

    public ReefFinderSubsystem() {

        if (!EnabledSubsystems.reef) {
            return;
        }

        // Initialize the CANRange sensor with the appropriate CAN ID
        rangeSensor = new CANrange(ReefFinderConstants.reefCANRangeID); // Replace '1' with the actual CAN ID of your sensor

        // Configure the sensor for short-distance detection
        configureCANRange();
        distance = rangeSensor.getDistance();
    }


  private void configureCANRange() {
    CANrangeConfiguration config = new CANrangeConfiguration();
    config.withProximityParams(new ProximityParamsConfigs()
        .withProximityThreshold(ReefFinderConstants.newProximityThreshold));
    config.withToFParams(new ToFParamsConfigs()
        .withUpdateMode(UpdateModeValue.ShortRangeUserFreq)
        .withUpdateFrequency(CanRangeConstants.newUpdateFrequency));
    rangeSensor.getConfigurator().apply(config);
  }


  public double getDistanceToTarget() {
    return distance.refresh().getValueAsDouble();
  }

  public boolean isTargetVisible() {
    return rangeSensor.getIsDetected().getValue(); 
  }


  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
