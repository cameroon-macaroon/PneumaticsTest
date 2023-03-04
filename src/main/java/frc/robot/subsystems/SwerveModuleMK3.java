// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//motor controls to import: feedbackDevice, NeutralMode, Remote Sensor Source,
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;

//talonFX to import: TalonFX Control Mode, TalonFX, TalonFX Configuration
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

//sensors to import: CANCoder, CANCoder Configuration
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

//math to import: Rotation2d, SwerveModuleState, Units
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
 import edu.wpi.first.math.util.Units;

public class SwerveModuleMK3 {

  // set constant PIDF values
  private static final double kDriveP = 15.0; //need to pick these values
  private static final double kDriveI = 0.01; //these are from last year
  private static final double kDriveD = 0.1;  //TODO: figure out what these should be 
  private static final double kDriveF = 0.2;

  // set constant PID values
  private static final double kAngleP = 1.0;
  private static final double kAngleI = 0.0;
  private static final double kAngleD = 0.0;

  // set ticks count
  private static double kEncoderTicksPerRotation = 4096;

  // create objects: driveMotor, angleMotor, canCoder, offset
  private TalonFX driveMotor;
  private TalonFX angleMotor;
  private CANCoder canCoder;
  private Rotation2d offset;

                        //Pass in: created objects
  public SwerveModuleMK3(TalonFX driveMotor, TalonFX angleMotor,CANCoder canCoder, Rotation2d offset) {
    
    //set passed in objects to created objects 
    this.driveMotor = driveMotor;
    this.angleMotor = angleMotor;
    this.canCoder = canCoder;
    this.offset = offset;

//angle motor settings
    // create angle TalonFX Configuration 
    TalonFXConfiguration angleTalonFXConfiguration = new TalonFXConfiguration();

    // set PID values
    angleTalonFXConfiguration.slot0.kP = kAngleP;
    angleTalonFXConfiguration.slot0.kI = kAngleI;
    angleTalonFXConfiguration.slot0.kD = kAngleD;

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
