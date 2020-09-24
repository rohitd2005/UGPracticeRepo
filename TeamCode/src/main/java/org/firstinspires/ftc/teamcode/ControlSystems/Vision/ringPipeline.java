package org.firstinspires.ftc.teamcode.ControlSystems.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ringPipeline extends OpenCvPipeline {

    private final int width = 640;
    private final int height = 480;

    private OpenCvCamera cam;
    private Mat hsvMat = new Mat();
    private int stackHeight = 0;

    private double areaTreshHold = 5.0; // random value needs to be tuned

    List<MatOfPoint> contours = new ArrayList<>();

    private final hsvArray lowYellowRange = new hsvArray(21 , 40 , 180); // tuned values for yellow low
    private final hsvArray highYellowRange = new hsvArray(38 , 255 , 255); // tuned values for yellow high

    public ringPipeline(HardwareMap hwmap){
        setupRingDetector(hwmap);
    }

    public void setupRingDetector(HardwareMap hwmap){
        int cameraId = hwmap.appContext.getResources().getIdentifier("cameraMonitorViewId" , "id" , hwmap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT , cameraId);
    }

    public void start(){
        cam.openCameraDevice();
        cam.setPipeline(this);
        cam.startStreaming(width , height , OpenCvCameraRotation.UPRIGHT);
    }

    public void end(){
        cam.stopStreaming();
        cam.closeCameraDevice();
    }

    @Override
    public Mat processFrame(Mat input) {
        contours.clear();
        Mat yellowMask = new Mat();
        Mat result = new Mat();
        Mat blurredMat = new Mat();

        Imgproc.pyrMeanShiftFiltering(input , blurredMat , 3 , 3);

        Imgproc.cvtColor(blurredMat , hsvMat , Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(hsvMat,hsvMat, new Size(3,3), 0);

        Core.inRange(hsvMat , lowYellowRange.getScalar() , highYellowRange.getScalar() , yellowMask);

        Imgproc.findContours(yellowMask , contours , new Mat() , Imgproc.RETR_LIST , Imgproc.CHAIN_APPROX_SIMPLE);

        Core.bitwise_and(input , input , result , yellowMask); // for tuning purposes

        MatOfPoint2f approximateCurve = new MatOfPoint2f();

        double greatestHeight = 0;

        if(contours.size() > 0){
            for(MatOfPoint m : contours){
                MatOfPoint2f m2 = new MatOfPoint2f(m.toArray());

                double distance = Imgproc.arcLength(m2 , true) * 0.02;
                Imgproc.approxPolyDP(m2, approximateCurve , distance , true);

                MatOfPoint points = new MatOfPoint(approximateCurve.toArray());

                Rect boundingRect = Imgproc.boundingRect(points);

                if(boundingRect.width * boundingRect.height > areaTreshHold) {

                    double startingPointx = boundingRect.x;
                    double startingPointy = boundingRect.y;

                    double endingPointX = startingPointx + boundingRect.width;
                    double endingPointY = startingPointy + boundingRect.height;

                    Scalar rectColor = new Scalar(0, 255, 0);
                    Imgproc.rectangle(input, new Point(startingPointx, startingPointy), new Point(endingPointX, endingPointY), rectColor, 2);

                    if(boundingRect.height > greatestHeight){
                        greatestHeight = boundingRect.height;
                    }
                }
            }
        }

        // random values tune later
        if(greatestHeight > 10){
            this.stackHeight = 4;
        }else if(greatestHeight < 10 && greatestHeight > 1){
            this.stackHeight = 1;
        }else{
            this.stackHeight = 0;
        }

        return input;

    }

    public int getStackHeight(){
        return this.stackHeight;
    }

    public String getActiveZone(){
        if(getStackHeight() == 4){
            return "Box C";
        }else if(getStackHeight() == 1){
            return "Box B";
        }else{
            return "Box A";
        }
    }

}
