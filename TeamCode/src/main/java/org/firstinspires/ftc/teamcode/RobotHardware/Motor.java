package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motor {
    private DcMotor motor;
    private double previousPos = 0;
    private double time = 0;
    private double previousReset = 0;
    private double previousTime = 0;
    private final double NANOSECONDS_PER_MIN = 6e+10;
    private final double cpr = 537.6;
    public static final double MAX_RPM = 340;
    private double rpm = 0;
    private double wheelDiameter = 0;

    public Motor(String name , HardwareMap hwmap){
        motor = hwmap.dcMotor.get(name);
    }

    public void resetMotor(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void RUN_USING_ENCODER(){
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getRpm(){
        time = System.currentTimeMillis() - previousReset;
        double deltaPos = motor.getCurrentPosition() - previousPos;
        double deltaTime = (System.nanoTime() - previousTime) / NANOSECONDS_PER_MIN;

        if(time > 20){
            rpm = (deltaPos / cpr) / deltaTime;
            previousPos = getCurrentPosition();
            previousReset = System.currentTimeMillis();
            previousTime = System.nanoTime();
        }

        return rpm;
    }

    public void setMode(DcMotor.RunMode mode){
        motor.setMode(mode);
    }

    public void setBreakMode(){
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setFloatMode(){
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public double getCurrPos(){
        return motor.getCurrentPosition();
    }

    public double getCurrPosInches(){
        return getCurrPos() / getTicksPerInch();
    }


    public void setPower(double power){
        motor.setPower(power);
    }

    public double getCurrentPosition(){
        return motor.getCurrentPosition();
    }


    public void setWheelDiameter(double wheelDiameter){  this.wheelDiameter = wheelDiameter; }

    public double getTicksPerInch(){
        return cpr / (wheelDiameter * Math.PI);
    }

    public void reverse(){
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

}
