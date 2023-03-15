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
  public static double frontLeftOffset = 50; //1 
  public static double frontRightOffset = 157;//0 
  public static double backLeftOffset = 0;//3
  public static double backRightOffset = 55; //2

  //set CAN ids for every module
  public static final int frontLeftDriveId = 10; 
  public static final int frontLeftCANCoderId = 11; 
  public static final int frontLeftSteerId = 9; 
  //put your can Id's here! 
  public static final int frontRightDriveId = 8; 
  public static final int frontRightCANCoderId = 14; 
  public static final int frontRightSteerId = 7; 
  //put your can Id's here! 
  public static final int backLeftDriveId = 4; 
  public static final int backLeftCANCoderId = 12; 
  public static final int backLeftSteerId = 3; 
  //put your can Id's here! 
  public static final int backRightDriveId = 6; 
  public static final int backRightCANCoderId = 13; 
  public static final int backRightSteerId = 5; 


  



  //set gyro 
  public static AHRS gyro = new AHRS(SPI.Port.kMXP); 

  //create Swerve Drive Kinematics 
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics( 
    new Translation2d(
      Units.inchesToMeters(10), 
      Units.inchesToMeters(10)
    ), 
    new Translation2d(
      Units.inchesToMeters(10), 
      Units.inchesToMeters(-10)
    ), 
    new Translation2d(
      Units.inchesToMeters(-10), 
      Units.inchesToMeters(10)
    ), 
    new Translation2d(
      Units.inchesToMeters(-10), 
      Units.inchesToMeters(-10)
    )
  ); 
    
      
  //create array of Modules
  private SwerveModuleMK3[] modules = new SwerveModuleMK3[] {
    
    new SwerveModuleMK3(new TalonFX(frontLeftDriveId), new TalonFX(frontLeftSteerId), new CANCoder(frontLeftCANCoderId), Rotation2d.fromDegrees(frontLeftOffset)), // Front Left
    new SwerveModuleMK3(new TalonFX(frontRightDriveId), new TalonFX(frontRightSteerId), new CANCoder(frontRightCANCoderId), Rotation2d.fromDegrees(frontRightOffset)), // Front Right
    new SwerveModuleMK3(new TalonFX(backLeftDriveId), new TalonFX(backLeftSteerId), new CANCoder(backLeftCANCoderId), Rotation2d.fromDegrees(backLeftOffset)), // Back Left
    new SwerveModuleMK3(new TalonFX(backRightDriveId), new TalonFX(backRightSteerId), new CANCoder(backRightCANCoderId), Rotation2d.fromDegrees(backRightOffset))  // Back Right
  }; 




  /** Creates a new SwerveDriveTrain. */
  public SwerveDriveTrain() {
     gyro.reset(); 
  }

  //Drive method 
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean calibrateGyro) {

    if(calibrateGyro){
      gyro.reset(); // recalibrates gyro offset 
    } 
                                                                                      //Rotation2d.fromDegrees(-gyro.getAngle())
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,Rotation2d.fromDegrees(-gyro.getAngle())) : new ChassisSpeeds(xSpeed, ySpeed, rot)); 

    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeed); 

    for(int i = 0; i < states.length; i++) { 
      SwerveModuleMK3 module = modules[i]; 
      SwerveModuleState state = states[i]; 
      SmartDashboard.putNumber(String.valueOf(i), module.getRawAngle()); 
      //below is a line to comment out from step 5 
      module.setDesiredState(state); 
      //SmartDashboard.putNumber("gyro Angle", gyro.getAngle()); 
    }
  }

  @Override 
  public void periodic() {
    // This method will be called once per scheduler run
  } 

  @Override 
  public void simulationPeriodic() {
    // This method will be called once per schedule run during simulation
  }
} 
