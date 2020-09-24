package org.firstinspires.ftc.teamcode.Resources;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

public class Imu {


        private double reltiveYaw = 0;
        private double lastAngle = 0;
        private final BNO055IMU imu;



        public Imu(String name, HardwareMap hardwareMap) {
            imu = hardwareMap.get(BNO055IMU.class, name);
            setParameters();
            Thread updateYaw = new Thread();
            updateYaw.start();
        }

        private void setParameters() {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.useExternalCrystal = true;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.pitchMode = BNO055IMU.PitchMode.WINDOWS;
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            imu.initialize(parameters);
        }

        public void updateRelativeYaw() {
            if (lastAngle > 90 && getAngles()[0] < 0) {
                reltiveYaw = 180 * Math.round(reltiveYaw / 180) + (180 + getAngles()[0]);
            } else if (lastAngle < -90 && getAngles()[0] > 0) {
                reltiveYaw = 180 * Math.round(reltiveYaw / 180) - (180 - getAngles()[0]);
            } else if (Math.abs(reltiveYaw) <= 180) {
                reltiveYaw = getAngles()[0];
            } else {
                reltiveYaw += getAngles()[0] - lastAngle;
            }
            lastAngle = getAngles()[0];
        }

        public double getRelativeYaw() {
            return reltiveYaw;
        }

        public double[] getAngles() {
            Quaternion quatAngles = imu.getQuaternionOrientation();

            double w = quatAngles.w;
            double x = quatAngles.x;
            double y = quatAngles.y;
            double z = quatAngles.z;

            // for the Adafruit IMU, yaw and roll are switched
            double roll = Math.atan2(2 * (w * x + y * z), 1 - (2 * (x * x + y * y))) * 180.0 / Math.PI;
            double pitch = Math.asin(2 * (w * y - x * z)) * 180.0 / Math.PI;
            double yaw = Math.atan2(2 * (w * z + x * y), 1 - (2 * (y * y + z * z))) * 180.0 / Math.PI;

            return new double[]{yaw, pitch, roll};
        }

        public String dataOutput() {
            return String.format("Yaw: %.3f  Pitch: %.3f  Roll: %.3f", getAngles()[0], getAngles()[1], getAngles()[2]);
        }
    }





