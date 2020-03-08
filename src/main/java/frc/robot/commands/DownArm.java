/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.Robot;

public class DownArm extends Command {
  public DownArm() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.arm);
  }

  // Called just before this Command runs the first time
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }
  
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
      if (Math.abs(Robot.arm.armEncoder.getDistance()) > RobotMap.encoderLowerLimit) {
        Robot.arm.liftMotor.set(RobotMap.downArmSpeedAboveLowerLimit);
      } else {
        Robot.arm.liftMotor.set(RobotMap.downArmSpeedBelowLowerLimit);
      }
    }
  
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
      return !Robot.oi.ArmDownButton.get();
    }
  
    // Called once after isFinished returns true
    @Override
    protected void end() {
      Robot.arm.liftMotor.set(0);
    }
  
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
