package org.firstinspires.ftc.teamcode.Mako;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mako.SubSystems.DriveTrain;


public class Teleop extends OpMode {

    DriveTrain dt;
    final double max_speed = 1.0;

    @Override
    public void init() {
        dt = new DriveTrain(hardwareMap);
        dt.setMotorZeroBehaviors();
        dt.resetAllMotors();

    }

    @Override
    public void loop() {
        double leftY = -gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;

        dt.setDriveTrainPowers(leftX , leftY , rightX , max_speed);

    }
}
