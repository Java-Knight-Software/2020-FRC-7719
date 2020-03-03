/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.RobotMap;
import frc.robot.commands.Reload;

/**
 * Add your docs here.
 */
public class Storage extends Subsystem {

  public PWMVictorSPX m1_sg = new PWMVictorSPX(RobotMap.m1_775);
  public Servo lservo = new Servo(4);
  public Servo rservo = new Servo(5);

  @Override
  public void initDefaultCommand() {

    setDefaultCommand(new Reload());

  }
}
