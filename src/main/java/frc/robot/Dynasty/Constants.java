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
    public Double dtLKp;
    public Double dtRKp;
    public Double dtLkV;
    public Double dtRkV;
    public Double dtLkA;
    public Double dtRkA;
    public Double dtLKd;
    public Double dtRKd;
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
        dtLKp = .1;
        dtRKp = .1;
        dtLKd = .1;
        dtRKd = .1;
        dtLeftFwdVint = 1.1;
        dtLeftRevVint = -1.1; 
        dtRightFwdVint = 1.1;
        dtRightRevVint = -1.1;
        maxVel = 12.0;
        maxTurn = 7.0;
        maxAccel = 2.0;
        baseWidth = 2.02;
        dtGkP = .01;
        dtLkV = .04;
        dtLkA = .004;
    }

}
