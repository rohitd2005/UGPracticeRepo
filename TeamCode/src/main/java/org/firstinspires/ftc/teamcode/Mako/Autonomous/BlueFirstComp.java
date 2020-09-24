package org.firstinspires.ftc.teamcode.Mako.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ControlSystems.Vision.ringPipeline;
import org.firstinspires.ftc.teamcode.Mako.SubSystems.DriveTrain;

public class BlueFirstComp extends LinearOpMode {
    DriveTrain makoDT = new DriveTrain(hardwareMap);
    private ringPipeline ringDetector = new ringPipeline(hardwareMap);
    private String activeZone = "Null";

    @Override
    public void runOpMode() throws InterruptedException {
        makoDT.setDriveControllerConstants(BlueConstants.BLUE_DRIVEKP , BlueConstants.BLUE_DRIVEKI , BlueConstants.BLUE_DRIVEKD);
        makoDT.setTurnControllerContstants(BlueConstants.BLUE_TURNKP , BlueConstants.BLUE_TURNKI , BlueConstants.BLUE_TURNKD);

        ringDetector.start();

        while(!opModeIsActive()){
            this.activeZone = ringDetector.getActiveZone();
            telemetry.addData("Stack Height: " , ringDetector.getStackHeight());
            telemetry.update();
        }

        waitForStart();

        makoDT.driveDistance(BlueConstants.BLUE_DRIVEFORWARDDIST , BlueConstants.DRIVEFORWARD_STOPSTATE , BlueConstants.DRIVEFORWARD_STARTIMEDIST);
        makoDT.strafe(BlueConstants.BLUE_STRAFEDIST , BlueConstants.STRAFE_STOPSTATE , BlueConstants.STRAFE_STARTTIMEDIST);
        //shoot rings vel pid
        makoDT.resetAllMotors();

        goToZone(activeZone);

    }

    public void goToZone(String activeZone){
        double strafeDistance;
        double forwardDistance;

        boolean boxC = false;

        if(activeZone.equals("Box A")){
            strafeDistance = BlueConstants.STRAFE_TO_BOX_A_DIST;
            forwardDistance = BlueConstants.DRIVE_FORWARD_TOLINE_BOX_A;
        }else if (activeZone.equals("Box B")){
            strafeDistance = 0;
            forwardDistance = BlueConstants.DRIVE_FORWARD_BOX_B_DIST;
        }else{
            strafeDistance = BlueConstants.STRAFE_TO_BOX_C_DIST;
            forwardDistance = BlueConstants.DRIVE_FORWARD_BOX_C_DIST;
            boxC = true;
        }

        if(strafeDistance != 0) makoDT.strafe(strafeDistance , BlueConstants.STRAFE_STOPSTATE , BlueConstants.STRAFE_STARTTIMEDIST);
        makoDT.driveDistance(forwardDistance , BlueConstants.DRIVEFORWARD_STOPSTATE , BlueConstants.DRIVEFORWARD_STARTIMEDIST);

        //drop wobble

        if(boxC) makoDT.driveDistance(BlueConstants.DRIVE_TOLINE_BOX_C , BlueConstants.DRIVEFORWARD_STOPSTATE , BlueConstants.DRIVEFORWARD_STARTIMEDIST);

        makoDT.stop();
        telemetry.addLine("Completed Autonomous");

    }



}
