// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDown extends CommandBase {
  
  final ElevatorSubsystem elevatorSubsystem;
  boolean isPressed;
  /** Creates a new ElevatorDown. */
  public ElevatorDown(ElevatorSubsystem m_elevatorSubsystem,boolean m_isPressed) {
    // Use addRequirements() here to declare subsystem dependencies.

    elevatorSubsystem = m_elevatorSubsystem;
    addRequirements(m_elevatorSubsystem);
    isPressed = m_isPressed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(elevatorSubsystem.bottomSwitch.get() && isPressed){
      elevatorSubsystem.turnOnMotor(.05);
      SmartDashboard.putString("State","Movint Down");
    }
    else{
      elevatorSubsystem.turnOnMotor(-.02);
      
      SmartDashboard.putString("State","Static");
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
