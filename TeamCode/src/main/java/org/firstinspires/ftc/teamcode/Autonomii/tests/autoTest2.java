package org.firstinspires.ftc.teamcode.Autonomii.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;
@Disabled
@Autonomous (name = "!testAuto2")

public class autoTest2 extends LinearOpMode {
    volatile int a = 0;
    DcMotor plate;
    DcMotor intake;
    DcMotorEx shooter;
    Servo lift;
    Servo camera;
    DistanceSensor distanceSensor = null;
    NormalizedColorSensor test_color;

    Ball[] storage = {new Ball(), new Ball(), new Ball()};

    double timer = 1;
    int plateTarget = 0;
    int ballsCollected = -1;
    boolean ballTaken = false;
    boolean full = false;
    boolean slowShoot = false;
    int toShoot = 0;
    double avgColor;
    double hue = 0;
    class Ball{
        int pos = 0;
        int color = 0;
    }
    public enum Intake_Sensor {
        EMPTY,
        ROTATE,
        OUTAKE

    }

    SampleMecanumDrive drive;
    Intake_Sensor intake_sensor = Intake_Sensor.EMPTY;
    double distance;

    Limelight3A limelight;

    LLResult llResult;
    int aprilTagID = 0;
    NormalizedRGBA colors;

    public void movePlate(int plateTarget){
        plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        plate.setPower(0.7);
        plate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        plate.setTargetPosition(plateTarget);
        plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    class stateMachine implements Runnable{

        @Override
        public void run() {
            while (opModeIsActive() && !isStopRequested()) {
                llResult = limelight.getLatestResult();
                if(llResult != null && llResult.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("ID", fr.getFiducialId());
                        aprilTagID = fr.getFiducialId();
                    }
                }

                telemetry.addData("aaa: ", 1);
                telemetry.update();
            }
        }
    }

    stateMachine obj = new stateMachine();

    @Override
    public void runOpMode() throws InterruptedException {
        Thread thread = new Thread(obj);


        drive = new SampleMecanumDrive(hardwareMap);

        plate = hardwareMap.get(DcMotor.class, "plate");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        lift = hardwareMap.get(Servo.class, "lift");
        camera = hardwareMap.get(Servo.class, "servo_camera");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        test_color = hardwareMap.get(NormalizedColorSensor.class, "color");

        limelight = hardwareMap.get(Limelight3A .class, "limelight");
        limelight.pipelineSwitch(0);

        Pose2d startingPose = new Pose2d(-51, -44, Math.toRadians(-125));
        TrajectorySequence DecodeAuto = drive.trajectorySequenceBuilder(startingPose)

                .waitSeconds(10)
                .build();

        drive.setPoseEstimate(startingPose);

        limelight.start();
        thread.start();
        waitForStart();
        sleep(500);
        drive.followTrajectorySequence(DecodeAuto);
        sleep(30000);
    }
}