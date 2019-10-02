/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Dynasty.Constants;
import frc.robot.subsystems.DtMain;
import frc.robot.subsystems.FeildPos;
import robot.pathfinder.core.RobotSpecs;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.trajectory.BasicTrajectory;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;
import robot.pathfinder.follower.Follower.DirectionSource;
import robot.pathfinder.follower.Follower.DistanceSource;
import robot.pathfinder.follower.Follower.Motor;
import robot.pathfinder.follower.Follower.TimestampSource;
import edu.wpi.first.wpilibj.Encoder;

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
  public static AHRS gyro = new AHRS(Port.kUSB1);
  public static FeildPos pos = new FeildPos(gyro);

  public static Encoder rightEncoder;
  public static Encoder leftEncoder;

  public static RobotSpecs robotSpecs;
  public static TrajectoryParams params;
  public static TankDriveTrajectory trajectory;

  public static final Motor L_MOTOR = pos::setLeftMotor;
  public static final Motor R_MOTOR = pos::setRightMotor;
  public static final DirectionSource GYRO = () -> {
    return Math.toRadians(pos.getHeading());
};
public static final DistanceSource L_DISTANCE_SOURCE = pos::getLeftDistance;
public static final DistanceSource R_DISTANCE_SOURCE = pos::getRightDistance;
public static final TimestampSource TIMESTAMP_SOURCE = Timer::getFPGATimestamp;
public static double distancePerPulse = 0.5 * Math.PI / 120;
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;
public static void init() {
  Ldrive1 = new TalonSRX(2);
  Ldrive2 = new TalonSRX(5);
  Rdrive1 = new TalonSRX(3);
  Rdrive2 = new TalonSRX(6);

  Rdrive1.setInverted(true);
  Rdrive2.setInverted(true);
  Ldrive2.follow(Ldrive1);
  Rdrive2.follow(Rdrive1);
  drivetrain = new DtMain();
  k = new Constants();
  gyro.setAngleAdjustment(90);

  rightEncoder = new Encoder(0, 1, false, EncodingType.k1X);
  leftEncoder = new Encoder(2, 3, false, EncodingType.k1X);
  rightEncoder.setReverseDirection(true);
  rightEncoder.setDistancePerPulse(distancePerPulse);
  leftEncoder.setDistancePerPulse(distancePerPulse);
  
  robotSpecs = new RobotSpecs(k.maxVel, k.maxAccel, k.baseWidth);
  params = new TrajectoryParams();
  params.waypoints = new Waypoint[] {
    new Waypoint(0.0, 0.0, Math.PI/2),
    new Waypoint(0, 5, Math.PI/2)
  };
  params.alpha = 10.0;
  params.isTank = true;
  trajectory = new TankDriveTrajectory(new BasicTrajectory(robotSpecs, params));
}
  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
