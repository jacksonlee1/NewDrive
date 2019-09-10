/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.subsystems.DtMain;
import frc.robot.Dynasty.Constants;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  public static TalonSRX Ldrive1;
  public static TalonSRX Ldrive2;
  public static TalonSRX Rdrive1;
  public static TalonSRX Rdrive2;
  public static DtMain drivetrain;
  public static Constants k;
  
  
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;
public static void init() {
  Ldrive1 = new TalonSRX(1);
  Ldrive2 = new TalonSRX(2);
  Rdrive1 = new TalonSRX(3);
  Rdrive2 = new TalonSRX(4);

  Ldrive2.set(ControlMode.Follower, 1);
  Rdrive2.set(ControlMode.Follower,3);
drivetrain = new DtMain();
 k = new Constants();
  
}
  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
