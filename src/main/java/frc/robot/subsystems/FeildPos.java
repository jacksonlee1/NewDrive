/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class FeildPos extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  AHRS gyro;
  public FeildPos(AHRS gyro){
    this.gyro = gyro;
  }
  public Double getHeading(){
    return gyro.getAngle();
  }
  public void setLeftMotor(double output) {
    if (Math.signum(output) > 0) {
      output += RobotMap.k.dtLeftFwdVint;
    } else {
      output += RobotMap.k.dtLeftRevVint;
    }
    RobotMap.Ldrive1.set(ControlMode.PercentOutput, output);
  }
  public void setRightMotor(double output) {
    if (Math.signum(output) > 0) {
      output += RobotMap.k.dtRightFwdVint;
    } else {
      output += RobotMap.k.dtRightRevVint;
    }
    RobotMap.Rdrive1.set(ControlMode.PercentOutput, output);
  }
  public Double getLeftDistance(){
    return  RobotMap.leftEncoder.getDistance();
  }
  public Double getRightDistance(){
    return RobotMap.rightEncoder.getDistance();
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
