/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.RobotMap;
import frc.robot.Robot;

public class Arm extends Subsystem {
  
  public Encoder armEncoder = new Encoder(RobotMap.armEncoderPorts[0], RobotMap.armEncoderPorts[1], false, Encoder.EncodingType.k4X);
  public WPI_VictorSPX liftMotor = new WPI_VictorSPX(RobotMap.liftMotor);

  public Servo brakeServo = new Servo(RobotMap.brakeServo);
  
  public Arm() {
    brakeServo.setAngle(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
