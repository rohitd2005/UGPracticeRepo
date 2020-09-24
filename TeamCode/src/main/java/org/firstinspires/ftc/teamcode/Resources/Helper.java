package org.firstinspires.ftc.teamcode.Resources;

import com.qualcomm.robotcore.util.Range;

public class Helper {
    public static double angleWrap(double angle){
        while(angle > Math.PI){
            angle -= (Math.PI * 2.0);
        }
        while(angle < -Math.PI){
            angle += (Math.PI * 2.0);
        }

        return angle;
    }

    public static void setInRange(double[] powerArray, double min, double max){
        for(int i = 0; i < powerArray.length; i ++){
            powerArray[i] = Range.clip(powerArray[i], min , max);
        }
    }

    public static void normalize(double[] powers){
        double max = 1;
        for(int i = 0 ; i < powers.length ; i ++){
            if(Math.abs(powers[i]) > max){
                max = Math.abs(powers[i]);
            }
        }

        if(max > 1){
            for(int i = 0 ; i < powers.length ; i ++){
                powers[i] /= max;
            }
        }
    }

}
