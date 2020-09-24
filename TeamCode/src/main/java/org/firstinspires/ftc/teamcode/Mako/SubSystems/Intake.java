package org.firstinspires.ftc.teamcode.Mako.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotHardware.Motor;

public class Intake {
    private Motor left;
    private Motor right;

    public Intake(HardwareMap hardwareMap){
        left = new Motor("leftIntake" , hardwareMap);
        right = new Motor("rightIntake" , hardwareMap);

        left.reverse();
    }

    public void intake(){
        left.setPower(1);
        right.setPower(1);
    }

    public void outtake(){
        left.setPower(-1);
        right.setPower(-1);
    }

}
