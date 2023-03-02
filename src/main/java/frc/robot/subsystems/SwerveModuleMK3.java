// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//motor controls to import: feedbackDevice, NeutralMode, Remote Sensor Source,
//talonFX to import: TalonFX Control Mode, TalonFX, TalonFX Configuration

//sensors to import: CANCoder, CANCoder Configuration
//math to import: Rotation2d, SwerveModuleState, Units

public class SwerveModuleMK3 {

  // set constant PIDF values

  // set constant PID values

  // set ticks count

  // create objects: driveMotor, angleMotor, canCoder, offset


  public SwerveModuleMK3() { //Pass in: created objects^^
    
    //set passed in objects to created objects 

//angle motor settings
    // create angle TalonFX Configuration 
    // set PID values

    // set remote Sensor to canCoder
    // set feedback Sensor to remote Sensor
    // configure created Configuration^^
    // set nuetral mode to break

//drive motor settings
    // creat drive Talon Configuration
    // set PIDF values

    // configure created Configureation^^
    // set nuetral mode to break

  }

  //method to get Angle (from CANCoder degrees)
  //method to get Raw Angle (from CANCoder double)

  //method to set Desired State

}
