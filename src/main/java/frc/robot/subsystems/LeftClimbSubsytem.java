// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LeftClimbSubsytem extends SubsystemBase {
  /** Creates a new LeftClimbSubsytem. */
  CANSparkMax leftClimb = new CANSparkMax(Constants.leftClimb,MotorType.kBrushless);
  RelativeEncoder leftEnc = leftClimb.getEncoder();
  SparkMaxPIDController leftPID = leftClimb.getPIDController();

  public LeftClimbSubsytem() {
    leftClimb.restoreFactoryDefaults();
    leftClimb.setIdleMode(IdleMode.kBrake);
    leftClimb.setInverted(false);
    leftClimb.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65410);
    leftPID.setP(.1);
    leftPID.setI(0);
    leftPID.setD(0);
    // leftPID.setOutputRange(-.2, .2);

    leftClimb.setOpenLoopRampRate(1); 
    leftClimb.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 65000);
    
  }



  public void set(double x)
  {
    leftClimb.set(x);
  }

  public void stop() {
    leftClimb.stopMotor();
  }

  public void setPos() {
    leftPID.setReference(leftEnc.getPosition(), CANSparkMax.ControlType.kPosition);
  }

  public double getVoltage() {
    return leftClimb.getOutputCurrent();
  }
  @Override
  public void periodic() {}
}
