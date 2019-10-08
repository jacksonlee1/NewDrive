/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.dynasty;

import java.util.HashMap;
import java.util.Map;

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
    public Map<String, Gains> gains;

    public Constants() {
        dtKp = 1.35;
        dtKd = 0.0419;
        dtLeftFwdVint = 2.39 / 12;
        dtLeftRevVint = 2.39 / 12;
        dtRightFwdVint = 2.39 / 12;
        dtRightRevVint = 2.39 / 12;
        maxVel = 12.0;
        maxTurn = 7.0;
        maxAccel = 2.0;
        baseWidth = 2.02;
        dtGkP = .01;
        dtKv = .433;
        dtKa = .068;

        gains = new HashMap<>();
        gains.put("LHF", new Gains(0.0858, 0.0386, 0.457, 0.0627, 2.34));
        gains.put("RHF", new Gains(0.0448, 0.0177, 0.430, 0.0305, 2.44));
        gains.put("LHR", new Gains(0.1870, 0.0900, 0.547, 0.1460, 1.89));
        gains.put("RHR", new Gains(0.1010, 0.0456, 0.541, 0.0743, 1.82));
        // low gear
        gains.put("LLF", new Gains(0, 0, 0, 0, 0));
        gains.put("RLF", new Gains(0, 0, 0, 0, 0));
        gains.put("LLR", new Gains(0, 0, 0, 0, 0));
        gains.put("RLR", new Gains(0, 0, 0, 0, 0));

    }

}
