/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Reload extends Command {
  public Reload() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.Storage);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if(Robot.OI.getP2Button(RobotMap.Y) == true) {

      Robot.Storage.m1_sg.set(0.50);

    }else if(Robot.OI.getP2Button(RobotMap.B) == true) {

      Robot.Storage.m1_sg.set(-0.50);

    }else {

      Robot.Storage.m1_sg.set(0);

    }

    if(Robot.OI.getP2Button(RobotMap.X) == true && Robot.OI.getP2Axis(RobotMap.RTrigger) < 0.5) {

      Robot.Storage.lservo.set(1.0);
      Robot.Storage.rservo.set(0.0);

    }else if(Robot.OI.getP2Button(RobotMap.X) == true && Robot.OI.getP2Axis(RobotMap.RTrigger) > 0.5){

      Robot.Storage.lservo.set(0.0);
      Robot.Storage.rservo.set(1.0);

    }//else {

     // Robot.Storage.Servo.set(0.0);

   // }
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
