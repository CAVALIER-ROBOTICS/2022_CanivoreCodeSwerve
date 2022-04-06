// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
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

    right.setInverted(TalonFXInvertType.CounterClockwise); //was Clockwise

    // left.follow(right);
    // left.setInverted(TalonFXInvertType.OpposeMaster); //was CounterClockwise - is jittering
    left.setInverted(TalonFXInvertType.Clockwise);
    

    right.setNeutralMode(NeutralMode.Coast);
    left.setNeutralMode(NeutralMode.Coast);

    right.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,1);
    left.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,1);

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


    double ff = 0.067;//.0592
    double p = 0.00;//.1
    int waitSec =1; // was 20 before...

    right.config_kF(0, ff, waitSec);
		right.config_kP(0, p, waitSec);

    // left.follow(right);

    left.config_kF(0, ff, waitSec);
		left.config_kP(0, p, waitSec);

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
    // SmartDashboard.putNumber("Fly Wheel", getVolicty());  //(rpm/600)*2048
    right.set(ControlMode.Velocity, (rpm/600)*2048);
    // right.set(ControlMode.PercentOutput, .45);

    left.set(ControlMode.Follower, Constants.rightShooterID);
    // left.set(ControlMode.Velocity, (rpm/600)*2048);
    // left.setVoltage(3);
    // right.setVoltage(3);

    // right.setVoltage(12);
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