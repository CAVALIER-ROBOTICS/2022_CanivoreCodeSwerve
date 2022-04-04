// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import com.fasterxml.jackson.core.json.DupDetector;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.HopperFloorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OutTakeCommand extends ParallelCommandGroup {
  /** Creates a new OutTakeCommand. */
  public OutTakeCommand(IntakeSubsystem intakeSub, HopperFloorSubsystem floorSub, KickerSubsystem kickSub, ShooterSubsystem shootSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new StartEndCommand(
        ()-> intakeSub.setIntakeMotor(-20), 
        ()-> intakeSub.stopIntake(),
        intakeSub),
      new StartEndCommand(
        ()-> floorSub.setFloor(-15),
        ()-> floorSub.stopFloor(), 
        floorSub),
      new StartEndCommand(
        ()-> kickSub.setKicker(-30), 
        ()-> kickSub.stopKicker(),
        kickSub),
      new StartEndCommand(
        ()-> shootSub.setShooter(-20), 
        () -> shootSub.setShooter(0), 
        shootSub));
  }
}
