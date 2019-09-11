/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.VelDrive;

/**
 * Add your docs here.
 */
public class DtMain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new VelDrive());
  }
  public void Drive(Double fPow, Double turnPow){

  }

  public void setLeftMotor(double output) {
    if (Math.signum(output) > 0) {
      output += RobotMap.k.dtLeftFwdVint;
    } else {
      output += RobotMap.k.dtLeftRevVint;
    }
    RobotMap.Ldrive1.set(ControlMode.PercentOutput, output);
  }
}
