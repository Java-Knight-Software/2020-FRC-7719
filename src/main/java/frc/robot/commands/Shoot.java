/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Shoot extends Command {
  public Shoot() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.Shooter);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    /////// Aim Assist Switch ///////

    if(Robot.OI.getP2Button(RobotMap.A) == true && Robot.OI.getP2Axis(RobotMap.RTrigger) > 0.5) {

      Robot.LimeLight.AimBot = false;      

  }else if(Robot.OI.getP2Button(RobotMap.A) == true && Robot.OI.getP2Axis(RobotMap.RTrigger) < 0.5) {

    Robot.LimeLight.AimBot = true;      

  }else {

    System.out.println(Robot.LimeLight.STEER_error);

  }


    //////////////////////////
    /////// Aim Assist ///////
    //////////////////////////

    if(Robot.LimeLight.AimBot == true) {//Robot.OI.getP2Button(RobotMap.A) == true) {

      Robot.LimeLight.tv = NetworkTableInstance.getDefault().getTable("limelight-javakni").getEntry("tv").getDouble(0);
      Robot.LimeLight.tx = NetworkTableInstance.getDefault().getTable("limelight-javakni").getEntry("tx").getDouble(0);
      Robot.LimeLight.ty = NetworkTableInstance.getDefault().getTable("limelight-javakni").getEntry("ty").getDouble(0);
      Robot.LimeLight.ta = NetworkTableInstance.getDefault().getTable("limelight-javakni").getEntry("ta").getDouble(0);
    
      Robot.LimeLight.dt = Timer.getFPGATimestamp() - Robot.LimeLight.STEER_lastTimestamp;

      //Robot.LimeLight.sensorPosition = Robot.LimeLight.encoder.getPosition() * Robot.LimeLight.kDriveTick2Feet;


      /////// Detecting Target ///////

      if (Robot.LimeLight.tv < 1.0)
      {
        Robot.LimeLight.m_LimelightHasValidTarget = false;
        Robot.LimeLight.m_LimelightDriveCommand = 0.0;
        Robot.LimeLight.m_LimelightSteerCommand = 0.0;
        return;
      }

      Robot.LimeLight.m_LimelightHasValidTarget = true;


      /////// Steering ///////

      if (Robot.LimeLight.STEER_error > -10.0 && Robot.LimeLight.STEER_error < 10.0 && Robot.LimeLight.STEER_error < -0.1 && Robot.LimeLight.STEER_error > 0.1) {

        Robot.LimeLight.STEER_errorSum += Robot.LimeLight.STEER_error * Robot.LimeLight.dt;

      }else if(Robot.LimeLight.STEER_error < -10.0 || Robot.LimeLight.STEER_error > 10.0) {

        Robot.LimeLight.STEER_errorSum = 0;

      }else if(Robot.LimeLight.STEER_error > -0.1 || Robot.LimeLight.STEER_error < 0.1) {

        Robot.LimeLight.STEER_errorSum = 0;

      }

      
      /////// Calculation ///////

      Robot.LimeLight.STEER_error = 0.0 - (-Robot.LimeLight.tx);
      Robot.LimeLight.STEER_errorRate = (Robot.LimeLight.STEER_error - Robot.LimeLight.STEER_lastError) / Robot.LimeLight.dt;
      Robot.LimeLight.STEER_outputPower = Robot.LimeLight.kp_STEER_K * Robot.LimeLight.STEER_error + Robot.LimeLight.ki_STEER_K * Robot.LimeLight.STEER_errorSum + Robot.LimeLight.kd_STEER_K * Robot.LimeLight.STEER_errorRate;


      /////// STEER MAX Speed Filtering ///////

      if (Robot.LimeLight.STEER_outputPower > Robot.LimeLight.MAX_STEER)
      {

        Robot.LimeLight.STEER_outputPower = Robot.LimeLight.MAX_STEER;

      }else if(Robot.LimeLight.STEER_outputPower < -Robot.LimeLight.MAX_STEER)
      {

        Robot.LimeLight.STEER_outputPower = -Robot.LimeLight.MAX_STEER;

      }

      Robot.Shooter.m3_st.set((Robot.LimeLight.STEER_outputPower));


      //////////////////////////
      /////// Manual Aim ///////
      //////////////////////////

    }else if(Robot.LimeLight.AimBot == false) {//Robot.OI.getP2Button(RobotMap.A) == false) {

      Robot.Shooter.m3_st.set(Robot.OI.getP2Axis(RobotMap.LeftX) * 0.5);

    }


    //////////////////////////////////////////////
    /////// Shooter Logic and Calculation ////////
    //////////////////////////////////////////////

      /////// Get Sensor Position ///////

      Robot.Shooter.sensorPosition = Robot.Shooter.encoder.getPosition() * Robot.Shooter.kDriveTick2Feet;


      /////// Get setPoint ///////

      if (Robot.OI.getP2Button(RobotMap.RBumper) == true || Robot.OI.getP1Button(RobotMap.LBumper) == true) {

        Robot.Shooter.setpoint = 999999999;

      }

  
      /////// PID ///////

      Robot.Shooter.error = Robot.Shooter.setpoint - Robot.Shooter.sensorPosition;
      Robot.Shooter.dt = Timer.getFPGATimestamp() - Robot.Shooter.lastTimestamp;
      Robot.Shooter.errorRate = (Robot.Shooter.error - Robot.Shooter.lastError) / Robot.Shooter.dt;
      Robot.Shooter.outputPower = Robot.Shooter.kP * Robot.Shooter.error + Robot.Shooter.kI * Robot.Shooter.errorSum + Robot.Shooter.kD * Robot.Shooter.errorRate;


      /////// Recovery Power ///////

      if(-Robot.Shooter.encoder.getVelocity() <= 0 && Robot.OI.getP2Button(RobotMap.RBumper) == false && Robot.OI.getP2Button(RobotMap.LBumper) == false) {

        Robot.Shooter.setpoint = 0;
        Robot.Shooter.encoder.setPosition(0.0);
        Robot.Shooter.outputPower = 0.0;

      }

      /////// Shooting ///////

      else if(Robot.OI.getP2Button(RobotMap.RBumper) == true) {

      if(Robot.Shooter.encoder.getVelocity() >= -3675 && Robot.Shooter.encoder.getVelocity() <= -4675) {

        Robot.Shooter.outputPower = -1.0;

      }else if(Robot.Shooter.encoder.getVelocity() >= -4675) {

        Robot.Shooter.outputPower = -0.85;

      }
    }
    
    
      /////// Passing ///////

    else if(Robot.OI.getP2Button(RobotMap.LBumper) == true) {
        
      if(Robot.Shooter.encoder.getVelocity() >= -800 && Robot.Shooter.encoder.getVelocity() <= -1300) {

        Robot.Shooter.outputPower = -0.45;

      }else if(Robot.Shooter.encoder.getVelocity() >= -1300) {

        Robot.Shooter.outputPower = -0.25;

      }

    }
  
  
      /////// Output to Motors ///////

      Robot.Shooter.m1_st.set(-Robot.Shooter.outputPower);
      Robot.Shooter.m2_st.set(Robot.Shooter.outputPower);
  
  
      /////// Update Last - Variables ///////

      Robot.Shooter.lastTimestamp = Timer.getFPGATimestamp();
      Robot.Shooter.lastError = Robot.Shooter.error;


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
