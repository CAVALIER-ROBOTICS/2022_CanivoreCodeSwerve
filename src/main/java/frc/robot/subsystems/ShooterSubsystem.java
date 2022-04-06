// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Limelight;

public class ShooterSubsystem extends SubsystemBase {

  WPI_TalonFX right = new WPI_TalonFX(Constants.rightShooterID);
  WPI_TalonFX left = new WPI_TalonFX(Constants.leftShootID);

  public ShooterSubsystem() {
    right.configFactoryDefault();
    left.configFactoryDefault();

    left.setInverted(TalonFXInvertType.Clockwise); //was CounterClockwise - is jittering
    
    right.setInverted(TalonFXInvertType.CounterClockwise); //was Clockwise

    right.setNeutralMode(NeutralMode.Coast);
    left.setNeutralMode(NeutralMode.Coast);

    right.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,20);
    left.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,20);

    // double voltage = 9;

    // right.configVoltageCompSaturation(voltage); // "full output" will now scale to 11 Volts for all control modes when enabled.
    // right.enableVoltageCompensation(true);

    // left.configVoltageCompSaturation(voltage); 
    // left.enableVoltageCompensation(true);

    // right.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 62000);
    // right.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 65310);
    // right.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 65100);
    // // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 65300);
    // // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 65300);
    // right.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 60050);
    // // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 65300);
    // // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 65300); 

    // left.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 62500);
    // left.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 65310);
    // left.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 65500);
    // // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 65300);
    // // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 65300);
    // left.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 60000);


    double ff = 0.0695;//.0592
    double p = 0.003;//.1
    double i = 0;
    double d = 0;
    right.config_kF(0, ff, 10);
		right.config_kP(0, p, 10);
		right.config_kI(0, i, 10);
    right.config_IntegralZone(0, 100);
		right.config_kD(0, d, 10);

    // left.follow(right);

    left.config_kF(0, ff, 10);
		left.config_kP(0, p, 10);
		left.config_kI(0, i, 10);
    left.config_IntegralZone(0, 100);
		left.config_kD(0, d, 10);

  }

  /**
   * 
   * @param a Velocity to be set
   */
  public void setShooterVelocity(double rpm)
  {
    // leftPID.setReference(rpm, ControlType.kVelocity);
    // rightPID.setReference(rpm, ControlType.kVelocity);
    // leftShooter.set(1000, ControlType.kVelocity);
    // leftShooter.set(a);
    // rightShooter.set(a);
    // SmartDashboard.putNumber("Fly Wheel", getVolicty());
    right.set(ControlMode.Velocity, (rpm/600)*2048);
    left.set(ControlMode.Follower, Constants.rightShooterID);
  }

  public void setShooter(double volt) {
    left.set(ControlMode.PercentOutput, volt);
    right.set(ControlMode.PercentOutput, volt);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("table RPM", Limelight.getRPM());
    SmartDashboard.putNumber("fly Wheel right", (right.getSelectedSensorVelocity() * 600) / 2048 );
    SmartDashboard.putNumber("fly Wheel left", (left.getSelectedSensorVelocity() * 600) / 2048 );
  }
}