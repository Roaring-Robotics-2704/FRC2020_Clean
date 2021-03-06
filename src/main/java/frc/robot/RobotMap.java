/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
  
  public static double spoolDiameter = 1.97/12.0;
  //Need to test, 0 is not the real value
  public static double armUpperLimit = 10932.25;

  //CAN
  public static int rightFrontMotor = 4;
  public static int rightBackMotor = 1;
  public static int leftFrontMotor = 3;
  public static int leftBackMotor = 2;
  public static int colorWheelMotor = 5;
  
  //DIO
  public static int[] driveEncoderPorts = {2, 3};
  public static int[] armEncoderPorts = {0, 1};
  
  //PWM
  public static int armMotor = 6;
  public static int brakeServo = 0;
  public static int cameraServo = 1;
}
