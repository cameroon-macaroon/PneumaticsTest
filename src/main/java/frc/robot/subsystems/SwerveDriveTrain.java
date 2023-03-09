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
  public static final int frontLeftDriveId = 8; 
  public static final int frontLeftCANCoderId = 6; 
  public static final int frontLeftSteerId = 7; 
  //put your can Id's here! 
  public static final int frontRightDriveId = 14; 
  public static final int frontRihtCANCoderId = 3; 
  public static final int frontRightSteerId = 13; 
  //put your can Id's here! 
  public static final int backLeftDriveId = 10; 
  public static final int backleftCANCoderId = 5; 
  public static final int backLeftSteerId = 9; 
  //put your can Id's here! 
  public static final int backRightDriveId = 12; 
  public static final int backRightCANCoderId = 4; 
  public static final int backRightSteerId = 11; 


  



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
    
    new SwerveModuleMK3(new TalonFX(1), new TalonFX(1), new CANCoder(1), Rotation2d.fromDegrees(1)), // Front Left 
    new SwerveModuleMK3(new TalonFX(2), new TalonFX(2), new CANCoder(2), Rotation2d.fromDegrees(2)), // Front Right 
    new SwerveModuleMK3(new TalonFX(3), new TalonFX(3), new CANCoder(3), Rotation2d.fromDegrees(3)), // Back Left 
    new SwerveModuleMK3(new TalonFX(4), new TalonFX(4), new CANCoder(4), Rotation2d.fromDegrees(4)), // Back Right 
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
      SmartDashboard.putNumber("gyro Angle", gyro.getAngle()); 
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
