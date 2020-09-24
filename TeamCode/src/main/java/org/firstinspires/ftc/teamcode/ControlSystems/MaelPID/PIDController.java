package org.firstinspires.ftc.teamcode.ControlSystems.MaelPID;

import com.qualcomm.robotcore.util.Range;

public class PIDController {
    private double KP;
    private double KI;
    private double KD;
    private double previousTime = 0;
    private double error;
    private double i = 0;
    private double d = 0;
    private double deltaTime = 0;
    private double previousError = 0;

    public PIDController(double KP , double KI , double KD){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
    }

    public double getOutput(double currPos , double targetPos){
        error = targetPos - currPos;
        double P = KP * error;
        deltaTime = System.currentTimeMillis() - previousTime;
        i += currPos > targetPos * 0.8 ? deltaTime * error : 0;
        double I = KI * i;
        d = (error - previousError) / deltaTime;
        double D = KD * d;

        previousTime = System.currentTimeMillis();
        previousError = error;

        return Range.clip(P + I + D , -1 , 1);
    }

    public void setConstants(double KP , double KI , double KD){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
    }

    public double getError(){
        return this.error;
    }

    public double getKP(){
        return this.KP;
    }

    public double getKI(){
        return this.KI;
    }

    public double getKd(){
        return KD;
    }

}
