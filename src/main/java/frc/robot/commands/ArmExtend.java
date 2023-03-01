// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmExtend extends CommandBase {
  private final ArmSubsystem subsystem;
  /** Creates a new ArmExtend. */
  public ArmExtend(ArmSubsystem armSubsystem) {
    subsystem = armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  final XboxController xbox = new XboxController (0);
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (xbox.getRightBumper()){
      subsystem.spinMotor(-.3);
    }
    else {
      subsystem.spinMotor(0);
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
