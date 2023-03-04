// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems; 


import edu.wpi.first.wpilibj2.command.SubsystemBase; 
import com.ctre.phoenix.motorcontrol.can.TalonFX; 
import com.ctre.phoenix.sensors.CANCoder;
import frc.robot.Constants; 
import edu.wpi.first.math.geometry.Translation2d; 
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.math.util.Units;  
import com.kauailabs.navx.frc.AHRS; 
//import edu.wpi.first.wpilibj.SerialPort; 
import edu.wpi.first.wpilibj.SPI; 
import edu.wpi.first.math.geometry.Rotation2d; 


public class SwerveDriveTrain extends SubsystemBase {

  //set constants: max speed, max angle speed, field calibration 
  public static final double kMaxSpeed = Units.feetToMeters(3.6); // 1 feet per second 
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second 
  public static double feildCalibration = 0; 

  //set offsets for every module 
  public static double frontLeftOffset = 45.0; 
  public static double frontRightOffset = 135.0; 
  public static double backLeftOffset = 135.0; 
  public static double backRightOffset = 65.0; 
  //set CAN ids for every module

  //set gyro

  //create Swerve Drive Kinematics

  //create array of Modules



  /** Creates a new SwerveDriveTrain. */
  public SwerveDriveTrain() {}

  //Drive method

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
