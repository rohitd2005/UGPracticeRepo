package org.firstinspires.ftc.teamcode.Odometry;

import org.firstinspires.ftc.teamcode.Resources.Imu;
import org.firstinspires.ftc.teamcode.RobotHardware.Motor;

public class TankOdometry implements Runnable {
    private Motor leftEncoder;
    private Motor rightEncoder;
    Imu imu;
    private double prevLeftPos = 0;
    private double prevRightPos = 0;
    private double heading = 0;
    private double x = 0;
    private double y = 0;
    private double trackWidth = 0;

    public TankOdometry(Imu imu , double trackWidth){
        this.imu = imu;
        this.trackWidth = trackWidth;
    }

    public TankOdometry(Imu imu){
        this.imu = imu;
    }

    public void update(){
        double leftChange = (leftEncoder.getCurrPosInches() - prevLeftPos);
        prevLeftPos = leftEncoder.getCurrPosInches();
        double rightChange = (rightEncoder.getCurrPosInches() - prevRightPos);
        prevRightPos = rightEncoder.getCurrPosInches();

        double theta = Math.toRadians(imu.getAngles()[0]);
        //double theta = (leftChange - rightChange) / trackWidth;
        double relativeY = (leftChange + rightChange) / 2.0;

        double currX = (relativeY * Math.sin(theta));
        double currY = (relativeY * Math.cos(theta));

        x += currX;
        y += currY;
        heading = theta;

    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getHeading(){
        return heading;
    }

    @Override
    public void run() {
        update();
    }



}
