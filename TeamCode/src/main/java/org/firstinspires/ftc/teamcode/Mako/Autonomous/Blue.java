package org.firstinspires.ftc.teamcode.Mako.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ControlSystems.PurePursuit.PurePursuit;
import org.firstinspires.ftc.teamcode.ControlSystems.PurePursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.ControlSystems.Vision.ringPipeline;
import org.firstinspires.ftc.teamcode.Mako.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Odometry.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.Resources.CurvePoint;
import org.firstinspires.ftc.teamcode.Resources.Point;

import java.util.ArrayList;
import java.util.List;

public class Blue extends LinearOpMode {

    DriveTrain makoRobot = new DriveTrain(hardwareMap);

    PurePursuit purePursuit = new PurePursuit(makoRobot , new ThreeWheelLocalizer(null , null , null , 0 , 0 , null));

    CurvePoint shootingPosFirst = new CurvePoint(new Point(0 , 100), .75 , 0 ,
                                        10 , 1 , 90 , 25);

    CurvePoint shootingPosition = new CurvePoint(new Point(45 , 100), .75 , 0 ,
                                                10 , 1 , 90 , 35);

    CurvePoint zoneAPositon = new CurvePoint(new Point(-25 , 125), .75 , 0 ,
                                                    10 , 1 , 90 , 35); // needs to be tuned

    CurvePoint zoneBPosition = new CurvePoint(new Point(25 , 155), .75 , 0 ,
                                                        10 , 1 , 90 , 35); // needs to be tuned


    CurvePoint zoneCPosition = new CurvePoint(new Point(-25 , 185), .75 , 0 ,
                                                                10 , 1 , 90 , 35); // needs to be tuned

    private String activeZone = "Null";

    private PurePursuitPath navToShootingPOS;

    private ringPipeline ringDetector;

    @Override
    public void runOpMode() throws InterruptedException {
        navToShootingPOS = new PurePursuitPath();
        navToShootingPOS.add(shootingPosFirst);
        navToShootingPOS.add(shootingPosition);

        ringDetector = new ringPipeline(hardwareMap);

        makoRobot.purePursuit = this.purePursuit;

        ringDetector.start();

        while(!opModeIsActive()){
            this.activeZone = ringDetector.getActiveZone();
            telemetry.addData("Stack Height" , ringDetector.getStackHeight());
            telemetry.addData("Active Zone" , ringDetector.getActiveZone());
            telemetry.update();
        }

        waitForStart();

        makoRobot.followPath(navToShootingPOS , 1000 , 3);
    }



}
