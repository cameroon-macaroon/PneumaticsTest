// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double DEFAULT_SPEED = 1000.0;
    public static final int CompressorPort = 0;
    public static final int fowardChannel = 1;
    public static final int reverseChannel = 0;
    public static final int SolenoidModule = 0;

    public static final int sparkMaxDeviceID = 2;
    public static final int topSwitchChannel = 1; 
    public static final int bottomSwitchChannel = 0; 

    public static final int armMotorChannel = 0; 
    public static final int coneSensorChannel = 2; 

    
  //set offsets for every module 
  public static double frontLeftOffset = 55; //1 
  public static double frontRightOffset = 150; 
  public static double backLeftOffset = 25;//3
  public static double backRightOffset =55; //2

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

  //set constants: max speed, max angle speed, field calibration 
  public static final double kMaxSpeed = Units.feetToMeters(3.6); // 1 feet per second 
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second 
  public static double feildCalibration = 0; 

  public static final double DEAD_ZONE = 0.0321;

}

