// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import frc.robot.Constants.SwerveK;

/** Add your docs here. */
public class Motor {

  private Motor() {}

  public static Motor getInstance() {
    return InstanceHolder.mInstance;
  }

  public TalonFX motor(
      int canid, NeutralMode mode, int pidslot, double[] pidf, boolean inverted, String canbus) {
    TalonFX motor = new TalonFX(canid, canbus);
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.setNeutralMode(mode);
    motor.config_kP(pidslot, pidf[0]);
    motor.config_kI(pidslot, pidf[1]);
    motor.config_kD(pidslot, pidf[2]);
    motor.config_kF(pidslot, pidf[3]);
    motor.setInverted(inverted);
    motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms);
    motor.configVelocityMeasurementWindow(SwerveK.velocityMeasAmount);
    motor.configVoltageCompSaturation(SwerveK.voltComp);
    motor.enableVoltageCompensation(true);
    motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 50, 0.1));
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 50, 0.1));
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, SwerveK.statusOneMeas);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, SwerveK.statusTwoMeas);
    return motor;
  }

  public CANCoder cancoder(int canid, double offset, String bus) {
    CANCoder encoder = new CANCoder(canid, bus);
    encoder.configFactoryDefault();
    encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    encoder.configAbsoluteSensorRange(SwerveK.absRange);
    encoder.configSensorDirection(SwerveK.kInvertCanCoder);
    encoder.configMagnetOffset(offset);
    // encoder.setPositionToAbsolute();

    return encoder;
  }

  private static class InstanceHolder {
    private static final Motor mInstance = new Motor();
  }
}
