/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI {

  /////// Declare Controller ///////

  public Joystick P1F310 = new Joystick(RobotMap.P1F310);
  public Joystick P2F310 = new Joystick(RobotMap.P2F310);


  /////// Get Controller Axis ///////
  
  public double getP1Axis (int axis) 
  {
    return P1F310.getRawAxis(axis);
  }
  public double getP2Axis (int axis)
  {
    return P2F310.getRawAxis(axis);
  }


  /////// Get Controller Button ///////
  
  public boolean getP1Button(int Button)
  {
    return P1F310.getRawButton(Button);
  }
  public boolean getP2Button(int Button)
  {
    return P2F310.getRawButton(Button);
  }


  /////// Get Controller Pov ///////

  public int getP1Dpad(int Dpad)
  {
    return P1F310.getPOV(Dpad);
  }
  public int getP2Dpad(int Dpad)
  {
    return P2F310.getPOV(Dpad);
  }
  
}
