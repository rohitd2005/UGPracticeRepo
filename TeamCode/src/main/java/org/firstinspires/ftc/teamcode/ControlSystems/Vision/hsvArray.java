package org.firstinspires.ftc.teamcode.ControlSystems.Vision;

import org.opencv.core.Scalar;

public class hsvArray {
    private int hue;
    private int sat;
    private int value;

    public hsvArray(int hue , int sat , int value) {
        this.hue = hue;
        this.sat = sat;
        this.value = value;
    }

    public Scalar getScalar(){
        return new Scalar(hue , sat , value);
    }

    public int getHue(){
        return this.hue;
    }

    public int getSat(){
        return sat;
    }

    public int getValue(){
        return value;
    }

    public void setHue(int hue){
        this.hue = hue;
    }

    public void setSat(int sat){
        this.sat = sat;
    }

    public void setValue(int value){
        this.value = value;
    }


}
