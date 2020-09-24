package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MaelServo {
    private Servo servo;
    private boolean prevState = false;
    private boolean taskState = true;

    public MaelServo(String servoName , HardwareMap hwmap){
        servo = hwmap.servo.get(servoName);
    }

    public void setPosition(double position){
        servo.setPosition(position);
    }

    public double getPosition(){
        return servo.getPosition();
    }

    public void setDirection(Servo.Direction direction){
        servo.setDirection(direction);
    }

    public Servo.Direction getDirection(){
        return servo.getDirection();
    }

    public boolean toggle(boolean pressed){
        boolean currState;
        if(pressed) currState = true;
        else{
            currState = false;
            if(prevState) taskState = !taskState;
        }
        prevState = currState;

        return taskState;
    }

}
