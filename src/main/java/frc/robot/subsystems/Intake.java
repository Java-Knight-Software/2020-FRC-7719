/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.Roll;

public class Intake extends Subsystem {

  public PWMVictorSPX m1_it = new PWMVictorSPX(RobotMap.m1_andy);
  //private PWMVictorSPX m2_it = new PWMVictorSPX(RobotMap.m2_775pro);

  //public SpeedController m_it = new SpeedControllerGroup(m1_it, m2_it);

  @Override
  public void initDefaultCommand() {

    setDefaultCommand(new Roll());

  }
}
