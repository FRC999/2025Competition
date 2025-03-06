// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.GPMConstants.IntakeConstants.PerimeterFinderConstants;
import frc.robot.Constants.GPMConstants.IntakeConstants.ReefFinderConstants;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.measure.Distance;


public class PerimeterFinderSubsystem extends SubsystemBase {

    private CANrange rangeSensorFront;
    private CANrange rangeSensorBack;
    StatusSignal<Distance> distanceFront;
    StatusSignal<Boolean> targetVisibleFront;
    StatusSignal<Distance> distanceBack;
    StatusSignal<Boolean> targetVisibleBack;


    public PerimeterFinderSubsystem() {

        if (!EnabledSubsystems.perimeter) {
            return;
        }

        // Initialize the CANRange sensor with the appropriate CAN ID
        rangeSensorFront = new CANrange(PerimeterFinderConstants.frontCANRangeID); // Replace '1' with the actual CAN ID of your sensor
        rangeSensorBack = new CANrange(PerimeterFinderConstants.frontCANRangeID); // Replace '1' with the actual CAN ID of your sensor

        // Configure the sensor for short-distance detection
        configureCANRange();
        distanceFront = rangeSensorFront.getDistance();
    }


  private void configureCANRange() {
    CANrangeConfiguration config = new CANrangeConfiguration();
    config.withProximityParams(new ProximityParamsConfigs()
        .withProximityThreshold(PerimeterFinderConstants.newProximityThreshold));
    config.withToFParams(new ToFParamsConfigs()
        .withUpdateMode(UpdateModeValue.ShortRangeUserFreq)
        .withUpdateFrequency(PerimeterFinderConstants.newUpdateFrequency));
    config.withFovParams(new FovParamsConfigs()
        .withFOVRangeX(PerimeterFinderConstants.perimeterFOVRangeX)
        .withFOVRangeY(PerimeterFinderConstants.perimeterFOVRangeY)
        );
    rangeSensorFront.getConfigurator().apply(config);
    rangeSensorBack.getConfigurator().apply(config);
  }


  public double getDistanceToTargetFront() {
    return distanceFront.refresh().getValueAsDouble();
  }
  public double getDistanceToTargetBack() {
    return distanceBack.refresh().getValueAsDouble();
  }

  public boolean isTargetVisibleFront() {
    return rangeSensorFront.getIsDetected().getValue(); 
  }
  public boolean isTargetVisibleBack() {
    return rangeSensorFront.getIsDetected().getValue(); 
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
