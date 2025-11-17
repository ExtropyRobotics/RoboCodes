package org.firstinspires.ftc.teamcode;/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous(name = "AprilTaertyuioithrdt8oytkturg" , group="!")
public class AprilTagCamera extends LinearOpMode
{
    OpenCvCamera camera;
    org.firstinspires.ftc.teamcode.AprilTagDetector aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    AprilTagDetection tag;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 21; // Tag 21

    AprilTagDetection tagOfInterest = null;


    NormalizedColorSensor color1 = null;

//
//    DcMotor motorCot = null;
//    DcMotor motorUmar1 = null;
//    DcMotor motorUmar2 = null;
//    Servo servo1, servo2, servo0;
//
//    SampleMecanumDrive drive;
//    Pose2d StartPos = new Pose2d(28,-61 , Math.toRadians(90));

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new org.firstinspires.ftc.teamcode.AprilTagDetector(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                telemetry.addLine("No error");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("error");
                telemetry.update();
            }
        });

        color1 = hardwareMap.get(RevColorSensorV3.class, "color1");

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
//        drive = new SampleMecanumDrive(hardwareMap);
//        drive.setPoseEstimate(StartPos);
//
//        motorCot = hardwareMap.get(DcMotor.class, "mM1");
//        motorUmar1 = hardwareMap.get(DcMotor.class , "mM2");
//        motorUmar2 = hardwareMap.get(DcMotor.class , "mM3");
//
//        servo0 = hardwareMap.get(Servo.class , "SV0");//fingerStanga
//        servo1 = hardwareMap.get(Servo.class , "SV1");//fingerDreapta
//        servo2 = hardwareMap.get(Servo.class , "SV2");//fingerHolder
//
//        motorCot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorCot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        motorUmar1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorUmar1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        motorUmar2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorUmar2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        motorUmar2.setDirection(DcMotor.Direction.REVERSE);
//        motorCot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        runEncoderManuta();

        double forwardAmount = 15.7;
        double backAmount = 13.7;

        int startTurn = -90;
        double turn = 145.5;
        double parkTurn = -144.5;

        int LMR = 30;
//
//        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(StartPos)
//                .forward(60.3)
//                .UNSTABLE_addTemporalMarkerOffset(-3 , ()->{
//                    ManutaJos(0);
//                    telemetry.addLine("jos");
//                    telemetry.update();
//                })
//                .turn(Math.toRadians(startTurn))
//                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
//                    Servo(false);
//                    goZeroManuta();
//                    telemetry.addLine("false");
//                    telemetry.update();
//                })
//                .forward(forwardAmount - 1)
//                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
//                    Servo(true);
//                    telemetry.addLine("true");
//                    telemetry.update();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.2 , ()->{
//                    int targetCot =   -100 ;//400; you go to viisoara if it doesnt work(it works)
//                    goTargetCot(targetCot);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.5 , ()->{
//                    int targetUmar =  -425   ;//-300
//                    goTargetUmar(targetUmar);
//                    telemetry.addLine("sus");
//                    telemetry.update();
//                })
//                .waitSeconds(0.5)
//                .back(backAmount+0.4)
//                .turn(Math.toRadians(turn))
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(-0.5 , ()->{
//                    goTargetCot(480);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->{
//                    goZeroManuta();
//                    Servo(false);
//                    telemetry.addLine("false");
//                    telemetry.update();
//                })
//                .turn(Math.toRadians(-turn))
//                .forward(forwardAmount+0.5)
//                .UNSTABLE_addTemporalMarkerOffset(-3 , ()->{
//                    ManutaJos(20);
//                    telemetry.addLine("jos");
//                    telemetry.update();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0 ,()->{
//                    goZeroManuta();
//                    Servo(true);
//                    telemetry.addLine("true");
//                    telemetry.update();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.1 , ()->{
//                    int targetCot =   -100 ;//400; you go to viisoara if it doesnt work(it works)
//                    goTargetCot(targetCot);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.5 , ()->{
//                    int targetUmar =  -425 ;//-300
//                    goTargetUmar(targetUmar);
//                    telemetry.addLine("sus");
//                    telemetry.update();
//                })
//                .waitSeconds(0.5)
//                .back(backAmount)
//                .turn(Math.toRadians(turn))
//                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
//                    goTargetCot(480);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.6 , ()->{
//                    goZeroManuta();
//                    Servo(false);
//                    telemetry.addLine("false");
//                    telemetry.update();
//                })
//                .waitSeconds(0.6)
//                .turn(Math.toRadians(-turn))
//                .forward(forwardAmount)
//                .UNSTABLE_addTemporalMarkerOffset(-2 , ()->{
//                    ManutaJos(50);
//                    telemetry.addLine("jos");
//                    telemetry.update();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0 ,()->{
//                    goZeroManuta();
//                    Servo(true);
//                    telemetry.addLine("true");
//                    telemetry.update();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.1 , ()->{
//                    int targetCot =   -100 ;//400; you go to viisoara if it doesnt work(it works)
//                    goTargetCot(targetCot);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.5 , ()->{
//                    int targetUmar =  -425 ;//-300
//                    goTargetUmar(targetUmar);
//                    telemetry.addLine("sus");
//                    telemetry.update();
//                })
//                .waitSeconds(0.5)
//                .back(backAmount)
//                .turn(Math.toRadians(turn))
//                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
//                    goTargetCot(480);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.6 , ()->{
//                    goZeroManuta();
//                    Servo(false);
//                    telemetry.addLine("false");
//                    telemetry.update();
//                })
//                .waitSeconds(0.6)
//                .UNSTABLE_addTemporalMarkerOffset(1 , ()->{
//                    ManutaJos(0);
//                    telemetry.addLine("jos");
//                    telemetry.update();
//                })
//                .waitSeconds(0.2)
//                .turn(Math.toRadians(parkTurn))//park
//                .build();
//
//        Wrist(false);

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0 && !isStarted())
            {
                tag = currentDetections.get(0);
                if(tag!=null)
                {   tagToTelemetry(tag);
                    telemetry.update();
                    break;}
            }
        }
        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            NormalizedRGBA colors = color1.getNormalizedColors();

            telemetry.addData("red ", colors.red);
            telemetry.addData("blue ", colors.blue);
            telemetry.addData("green ", colors.green);
            telemetry.update();
        }
//        if(tagOfInterest!=null)
//        {
//            if(tag.id == 19) {
//                LMR = 40;
//            }
//            if(tag.id == 18) {
//                LMR = 1;
//            }
//            if(tag.id == 17) {
//                LMR = -30;
//            }
//        }
//        else {
//            LMR = 1;
//        }
//        Trajectory park = drive.trajectoryBuilder(StartPos)
//                .forward(LMR)
//                .build();
//        drive.followTrajectorySequence(trajectory);
//        drive.setPoseEstimate(StartPos);
//        drive.followTrajectory(park);



    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        telemetry.update();
    }
//    public void ManutaJos(int targetCot)
//    {
//        int targetUmar =  -1200;//-900
//        goTargetCot(targetCot);
//        goTargetUmar(targetUmar);
//    }
//    public void goZeroManuta(){
//        motorCot.setPower(0);
//        motorUmar1.setPower(0);
//        motorUmar2.setPower(0);
//    }
//    public void runEncoderManuta()
//    {
//        motorUmar1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorUmar2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorCot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorUmar1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorUmar2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorCot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//    public void goTargetUmar(int target)
//    {
//        double power = 0.3;
//        motorUmar1.setTargetPosition(target);
//        motorUmar2.setTargetPosition(target);
//        motorUmar1.setPower(power);
//        motorUmar2.setPower(power);
//        motorUmar1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorUmar2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//    public void goTargetCot(int target)
//    {
//        motorCot.setTargetPosition(target);
//        motorCot.setPower(0.5);
//        motorCot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//    public void Servo(boolean x)
//    {
//        if(x){
//            servo0.setPosition(0.2);//closed
//            servo1.setPosition(0.4);
//        }
//        else{
//            servo0.setPosition(0.6);//opened
//            servo1.setPosition(0);
//        }
//    }
//    public void Wrist(boolean x)
//    {
//        if(x){
//            servo2.setPosition(0);//blue up
//        }
//        else{
//            servo2.setPosition(0.7);//blue down
//        }
//    }
}