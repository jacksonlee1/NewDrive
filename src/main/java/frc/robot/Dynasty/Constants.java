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
        dtLKp = .00000;
        dtRKp = .00000;
        dtLKd = .0;
        dtRKd = .0;
        dtLeftFwdVint = 0;
        dtLeftRevVint = 0; 
        dtRightFwdVint = 0;
        dtRightRevVint = 0;
        maxVel = 12.0;
        maxTurn = 7.0;
        maxAccel = 2.0;
        baseWidth = 2.02;
        dtGkP = .0000;
        dtLkV = .483;
        dtLkA = .00;
    }

}
