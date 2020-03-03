/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.Move;

/**
 * Add your docs here.
 */
public class LimeLight extends Subsystem {


  public double setpoint = 0;
  public double sensorPosition = 0;
  public double dt = 0;
  public double LeftStickY;
  public double LeftStickX;
  public double STEER_error;
  public double DRIVE_error;
  public double STEER_errorRate;
  public double DRIVE_errorRate;
  public double DRIVE_outputPower;
  public double STEER_outputPower;
  public double m_LimelightDriveCommand = 0.0;
  public double m_LimelightSteerCommand = 0.0;
  public double STEER_errorSum = 0;
  public double STEER_lastTimestamp = 0;
  public double STEER_lastError = 0;
  public double DRIVE_errorSum = 0;
  public double DRIVE_lastTimestamp = 0;
  public double DRIVE_lastError = 0;
  public double tv;
  public double tx;
  public double ty;
  public double ta;

  public boolean isAutonomous = false;
  public boolean m_LimelightHasValidTarget = false;
  public boolean DriveSwitch = false;
  public boolean AimBot = false;

  public String m_autoSelected;

  public final double kp_STEER_K = 0.03;                    // how hard to turn toward the target
  public final double kp_DRIVE_K = 0.2;                    // how hard to drive fwd toward the target
  public final double DESIRED_TARGET_AREA = 1.0;        // Area of the target when the robot reaches the wall
  public final double MAX_DRIVE = 0.5;                   // Simple speed limit so we don't drive too fast
  public final double MAX_STEER = 0.75;
  public final double ki_STEER_K = 0.01;
  public final double ki_DRIVE_K = 0.1;
  public final double kd_STEER_K = 0.006;
  public final double kd_DRIVE_K = 0.04;
  public final double kDriveTick2Feet = 10.0 / 128 * 6 * Math.PI / 10;

  public final String kDefaultAuto = "Default";
  public final String kCustomAuto = "My Auto";

  public final SendableChooser<String> m_chooser = new SendableChooser<>();

  @Override
  public void initDefaultCommand() {

    //setDefaultCommand(new Shoot());
    setDefaultCommand(new Move());

    setpoint = 0;

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);

  }
}
