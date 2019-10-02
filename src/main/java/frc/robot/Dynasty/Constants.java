/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Dynasty;



/**
 * Add your docs here.
 */
public class Constants {
    public Double dtKp;
    public Double dtKv;
    public Double dtKa;
    public Double dtKd;
    public double dtGkP;
    public double dtLeftFwdVint;
    public double dtLeftRevVint;
    public double dtRightFwdVint;
    public double dtRightRevVint;
    public Double maxVel;
    public Double maxAccel;
    public Double maxTurn;
    public Double baseWidth;

    public Constants(){
        dtKp = 1.35;
        dtKd = 0.0419;
        dtLeftFwdVint = 2.39/12;
        dtLeftRevVint =  2.39/12; 
        dtRightFwdVint =  2.39/12;
        dtRightRevVint = 2.39/12;
        maxVel = 12.0;
        maxTurn = 7.0;
        maxAccel = 2.0;
        baseWidth = 2.02;
        dtGkP = .01;
        dtKv = .433;
        dtKa = .068;
    }

}
