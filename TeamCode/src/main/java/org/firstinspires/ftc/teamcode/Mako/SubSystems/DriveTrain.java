package org.firstinspires.ftc.teamcode.Mako.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ControlSystems.MaelPID.PIDController;
import org.firstinspires.ftc.teamcode.ControlSystems.PurePursuit.PurePursuit;
import org.firstinspires.ftc.teamcode.ControlSystems.PurePursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.Odometry.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.Resources.Helper;
import org.firstinspires.ftc.teamcode.Resources.Imu;
import org.firstinspires.ftc.teamcode.RobotHardware.Motor;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class DriveTrain {
    private Motor tl;
    private Motor bl;
    private Motor tr;
    private Motor br;
    private double STRAFE_MULTIPLIER = 1.5;
    private double TURN_MULTIPLIER = -1.0;
    private double driveKp, driveKi , driveKd = 0;
    private double turnKp , turnKi , turnKd = 0;
    public PIDController driveController;
    public PIDController turnController;
    public PurePursuit purePursuit;
    public ThreeWheelLocalizer odometry;
    public Imu imu;

    public DriveTrain(HardwareMap hardwareMap){
        tl = new Motor("topLeft" , hardwareMap);
        bl = new Motor("backRight" , hardwareMap);
        tr = new Motor("topRight" , hardwareMap);
        br = new Motor("backRight" , hardwareMap);

        imu = new Imu("imu" , hardwareMap);
        driveController = new PIDController(driveKp , driveKi , driveKd);
        turnController = new PIDController(turnKp , turnKi , turnKd);

        tl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        tl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tl.setBreakMode();
        bl.setBreakMode();
        tr.setBreakMode();
        br.setBreakMode();

    }


    public void followPath(PurePursuitPath path , double stopTime , double startTimerDistance){
        double startTimer = 0;
        double stopState = 0;

        while(stopState < stopTime){
            purePursuit.followPath(path);

            double distanceToEnd = Math.hypot(purePursuit.odometry.getX() - path.get(path.size() - 1).x ,
                    purePursuit.odometry.getY() - path.get(path.size() - 1).y);

            if(distanceToEnd <= startTimerDistance){
                stopState = System.currentTimeMillis() - startTimer;
            }else{
                startTimer = System.currentTimeMillis();
            }
        }

        stop();

    }



    public void driveDistance(double distance , double stopTime , double startStopDistance){
        double startTime = 0;
        double stopState = 0;

        while(stopState < stopTime){

            double currPosInches = bl.getCurrPosInches();
            double power = driveController.getOutput(currPosInches , distance);

            tl.setPower(power);
            bl.setPower(power);
            tr.setPower(power);
            bl.setPower(power);

            if(driveController.getError() <= startStopDistance){
                stopState = System.currentTimeMillis() - startTime;
            }else{
                startTime = System.currentTimeMillis();
            }

        }

        stop();
    }

    public void strafe(double distance , double stopTime , double startStopDistance){
        double startTime = 0;
        double stopState = 0;

        while(stopState < stopTime){

            double currPosInches = bl.getCurrPosInches();
            double power = driveController.getOutput(currPosInches , distance);

            tl.setPower(power);
            bl.setPower(-power);
            tr.setPower(-power);
            bl.setPower(power);


            if(driveController.getError() <= startStopDistance){
                stopState = System.currentTimeMillis() - startTime;
            }else{
                startTime = System.currentTimeMillis();
            }

        }

        stop();
    }


    public void turn(double targetAngle , double stopTime , double startStopAngle ){
        double startTimer = 0;
        double stopState = 0;

        while(stopState < stopTime){
            double turnPower = turnController.getOutput(imu.getAngles()[0] , targetAngle) * TURN_MULTIPLIER;

            tl.setPower(turnPower);
            bl.setPower(turnPower);
            tr.setPower(turnPower);
            br.setPower(turnPower);

            if(turnController.getError() <= startStopAngle){
                stopState = System.currentTimeMillis() - startTimer;
            }else{
                startTimer = System.currentTimeMillis();
            }
        }
    }



    public void setDriveTrainPowers(double move_x , double move_y , double move_turn , double movementSpeed) {
        move_x = Range.clip(move_x, -movementSpeed, movementSpeed);
        move_y = Range.clip(move_y, -movementSpeed, movementSpeed);

        double tlPower = move_y + move_x - move_turn;
        double blPower = move_y - move_x - move_turn;
        double trPower = move_y - move_x + move_turn;
        double brPower = move_y + move_x + move_turn;

        List<Double> powerList = Arrays.asList(tlPower, blPower, trPower, brPower);
        double max = Collections.max(powerList);

        if (max > 1) {
            for (int i = 0; i < powerList.size() - 1; i++) {
                double current = powerList.get(i);
                powerList.set(i, current / max);
            }
        }

        tl.setPower(powerList.get(0));
        bl.setPower(powerList.get(0));
        tr.setPower(powerList.get(0));
        br.setPower(powerList.get(0));
    }



    public void fieldCentric(double xSpeed, double ySpeed, double turn , double movementSpeed){
        double angle = Math.atan2(ySpeed , xSpeed) - Math.toDegrees(odometry.getHeading());

        double speed = Math.hypot(xSpeed , ySpeed);

        double move_x = speed * Math.cos(angle);
        double move_y = speed * Math.sin(angle);

        move_x = Range.clip(move_x , -movementSpeed , movementSpeed);
        move_y = Range.clip(move_y , -movementSpeed , movementSpeed);

        setDriveTrainPowers(move_x , move_y , turn , movementSpeed );
    }

    public void stop(){
        tl.setPower(0);
        bl.setPower(0);
        tr.setPower(0);
        br.setPower(0);
    }

    public void setMotorZeroBehaviors(){
        bl.setBreakMode();
        br.setBreakMode();
        tl.setBreakMode();
        bl.setBreakMode();
    }

    public void resetAllMotors(){
        bl.resetMotor();
        tl.resetMotor();
        tr.resetMotor();
        br.resetMotor();
    }

    public void setDriveControllerConstants(double kp , double ki , double kd){
        this.driveKp = kp;
        this.driveKi = ki;
        this.driveKd = kd;
        driveController.setConstants(driveKp , driveKi , driveKd);
    }

    public void setTurnControllerContstants(double kp , double ki , double kd){
        this.turnKi = ki;
        this.turnKp = kp;
        this.turnKd = kd;
        turnController.setConstants(turnKp , turnKi , turnKd);
    }

}
