/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.RobotMap; 

/**
 * Add your docs here.
 */
public class CameraServo extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  public Servo cameraServo = new Servo(RobotMap.cameraServo);
  public int stage = 1;

  public CameraServo() {
    cameraServo.setAngle(0);
  }

  public void changeStage(){
    switch(stage){
      
      case 1: 
      cameraServo.setAngle(RobotMap.stage1Angle);
      //System.out.println("case 1");
      break;
      
      case 2:
      cameraServo.setAngle(RobotMap.stage2Angle);
      //System.out.println("case 2");
      break;
      
      case 3:
      cameraServo.setAngle(RobotMap.stage3Angle);
      //System.out.println("case 3");
      break;
     
      default: 
      if (stage > 3){
      stage = 3;
      }
      if (stage < 0){
      stage = 0;
      }
      break;

    }
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
