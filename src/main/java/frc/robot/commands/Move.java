
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

public class Move extends Command {
  public Move() {

    requires(Robot.LimeLight);

  }
  
  @Override
  protected void initialize() {

  }

  @Override
  protected void execute() {

    if(Robot.LimeLight.isAutonomous == false && Robot.LimeLight.DriveSwitch == false)
    {

      ////////////////////////////
      /////// Manual Drive ///////
      ////////////////////////////

    Robot.LimeLight.LeftStickY = Robot.OI.getP1Axis(RobotMap.LeftY);
    Robot.LimeLight.LeftStickX = Robot.OI.getP1Axis(RobotMap.LeftX);

    if(Robot.OI.getP1Button(RobotMap.RBumper) == true) {

      /////// Boost ///////

    Robot.DriveTrain.setPower(-Robot.LimeLight.LeftStickY, Robot.LimeLight.LeftStickX);

    }else {

    Robot.DriveTrain.setPower(-Robot.LimeLight.LeftStickY * 0.50, Robot.LimeLight.LeftStickX * 0.75);

    }

  }else if(Robot.LimeLight.isAutonomous == true || Robot.OI.getP1Button(RobotMap.LBumper) == true)
    {

      ////////////////////////////
      /////// Auto-Drive /////////
      ////////////////////////////

      
        /////// Get Data ///////

        Robot.LimeLight.tv = NetworkTableInstance.getDefault().getTable("limelight-javakni").getEntry("tv").getDouble(0);
        Robot.LimeLight.tx = NetworkTableInstance.getDefault().getTable("limelight-javakni").getEntry("tx").getDouble(0);
        Robot.LimeLight.ty = NetworkTableInstance.getDefault().getTable("limelight-javakni").getEntry("ty").getDouble(0);
        Robot.LimeLight.ta = NetworkTableInstance.getDefault().getTable("limelight-javakni").getEntry("ta").getDouble(0);
      
        Robot.LimeLight.dt = Timer.getFPGATimestamp() - Robot.LimeLight.STEER_lastTimestamp;

        //Robot.LimeLight.sensorPosition = Robot.LimeLight.encoder.getPosition() * Robot.LimeLight.kDriveTick2Feet;
        

        /////////////////////////////////////
        /////// Logic and Calculation ///////
        /////////////////////////////////////


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

        if (Robot.LimeLight.STEER_error > -10.0 && Robot.LimeLight.STEER_error < 10.0 && Robot.LimeLight.STEER_error < -2 && Robot.LimeLight.STEER_error > 2) {

          Robot.LimeLight.STEER_errorSum += Robot.LimeLight.STEER_error * Robot.LimeLight.dt;

        }else if(Robot.LimeLight.STEER_error < -10.0 || Robot.LimeLight.STEER_error > 10.0) {

          Robot.LimeLight.STEER_errorSum = 0;

        }else if(Robot.LimeLight.STEER_error > -2.0 || Robot.LimeLight.STEER_error < 2.0) {

          Robot.LimeLight.STEER_errorSum = 0;

        }

        
        /////// Calculation ///////

        Robot.LimeLight.STEER_error = 0.0 - (-Robot.LimeLight.tx);
        Robot.LimeLight.DRIVE_error = 2.0 - (-Robot.LimeLight.ty);
        Robot.LimeLight.STEER_errorRate = (Robot.LimeLight.STEER_error - Robot.LimeLight.STEER_lastError) / Robot.LimeLight.dt;
        Robot.LimeLight.DRIVE_errorRate = (Robot.LimeLight.DRIVE_error - Robot.LimeLight.DRIVE_lastError) / Robot.LimeLight.dt;
        Robot.LimeLight.STEER_outputPower = Robot.LimeLight.kp_STEER_K * Robot.LimeLight.STEER_error + Robot.LimeLight.ki_STEER_K * Robot.LimeLight.STEER_errorSum + Robot.LimeLight.kd_STEER_K * Robot.LimeLight.STEER_errorRate;


        /////// STEER MAX Speed Filtering ///////

        if (Robot.LimeLight.STEER_outputPower > Robot.LimeLight.MAX_DRIVE)
        {

          Robot.LimeLight.STEER_outputPower = Robot.LimeLight.MAX_DRIVE;
        //m_LimelightSteerCommand = STEER_outputSpeed;

        }else if(Robot.LimeLight.STEER_outputPower < -Robot.LimeLight.MAX_DRIVE)
        {

          Robot.LimeLight.STEER_outputPower = -Robot.LimeLight.MAX_DRIVE;

        }

        if(Robot.LimeLight.STEER_error < -2.0 && Robot.LimeLight.STEER_error > 2.0)
        {
          return;
        }


        /////// Integral Increment ///////

        if (Robot.LimeLight.DRIVE_error < -2.5) {

          Robot.LimeLight.DRIVE_errorSum += Robot.LimeLight.DRIVE_error * Robot.LimeLight.dt;

        }


        /////// Drive Maju ///////

        Robot.LimeLight.DRIVE_outputPower = Robot.LimeLight.kp_DRIVE_K * -Robot.LimeLight.DRIVE_error + Robot.LimeLight.ki_DRIVE_K * Robot.LimeLight.DRIVE_errorSum + Robot.LimeLight.kd_DRIVE_K * Robot.LimeLight.DRIVE_errorRate;


        /////// DRIVE MAX Speed Filtering ///////

        if (Robot.LimeLight.DRIVE_outputPower > Robot.LimeLight.MAX_DRIVE)
        {
          Robot.LimeLight.DRIVE_outputPower = Robot.LimeLight.MAX_DRIVE;
        }


        /////// Set power ///////

        //Robot.LimeLight.setPower(Robot.LimeLight.DRIVE_outputPower, Robot.LimeLight.STEER_outputPower); 
        //Robot.Shooter.m3_st.set(Robot.LimeLight.STEER_outputPower); 


        /////// Update Varibale ///////

        Robot.LimeLight.STEER_lastError = Robot.LimeLight.STEER_error;
        Robot.LimeLight.STEER_lastTimestamp = Timer.getFPGATimestamp();
        Robot.LimeLight.DRIVE_lastError = Robot.LimeLight.DRIVE_error;
        Robot.LimeLight.DRIVE_lastTimestamp = Timer.getFPGATimestamp();

    }

  }

  @Override
  protected boolean isFinished() {

    return false;

  }

  @Override
  protected void end() {

  }

  @Override
  protected void interrupted() {

  }

}
