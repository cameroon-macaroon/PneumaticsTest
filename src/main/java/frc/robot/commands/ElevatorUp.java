// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class ElevatorUp extends CommandBase {
  final XboxController xbox = new XboxController(0);
  final ElevatorSubsystem subsystem;
  /** Creates a new ElevatorUp. */
  public ElevatorUp(ElevatorSubsystem elevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    subsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(subsystem.topSwitch.get() && xbox.getYButton()){
      subsystem.turnOnMotor(-.12);
    }
    else{
      subsystem.turnOnMotor(-.02);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
