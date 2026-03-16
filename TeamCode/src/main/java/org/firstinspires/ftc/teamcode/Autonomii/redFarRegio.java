package org.firstinspires.ftc.teamcode.Autonomii;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "!redFar")

public class redFarRegio extends LinearOpMode {
    DcMotorEx plateEncoder;
    CRServo plateServoLeft;
    CRServo plateServoRight;
    DcMotorEx intake;
    DcMotorEx outtake;
    DcMotorEx outtake2;
    NormalizedColorSensor colorSensor;
    DistanceSensor distanceSensor;
    DcMotorEx turret = null;
    Limelight3A limelight;
    Pose2d startingPose = new Pose2d(58, -10, Math.toRadians(180));

    int sumPlate = 0;
    int encoderPos = 0;
    double plateError = 0;
    ElapsedTime timer = new ElapsedTime();
    int plateTargetPosition = 0;
    double platePow = 1;
    double maxPlatePower = 0.6;
    double plateP = 0.0002;
    double plateI = 0.00000;
    double plateD = 0.0008;

    class autoThread implements Runnable {
        @Override
        public void run() {

            while (opModeIsActive() && !isStopRequested()) {
//                double plateDt = timer.seconds();
//
//                encoderPos = -plateEncoder.getCurrentPosition();
//                plateError = plateTargetPosition - encoderPos;
//                sumPlate += (int) (plateError * plateDt);
//                double deriv = (plateTargetPosition - encoderPos - plateError);
//
//                platePow = plateP*plateError + plateI*sumPlate + deriv* plateD;
//
//                if(platePow < -maxPlatePower) platePow = -maxPlatePower;
//                if(platePow > maxPlatePower) platePow = maxPlatePower;
//
//                plateServoRight.setPower(platePow);
//                plateServoLeft.setPower(platePow);
            }
        }
    }

    autoThread obj = new autoThread();

    @Override
    public void runOpMode(){
        Thread thread = new Thread(obj);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        plateServoLeft = hardwareMap.get(CRServo.class, "plateServoLeft");
        plateServoRight = hardwareMap.get(CRServo.class, "plateServoRight");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        plateEncoder = hardwareMap.get(DcMotorEx.class, "intake");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);

        TrajectorySequence DecodeAuto= drive.trajectorySequenceBuilder(startingPose)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    outtake.setVelocity(1600);
                    outtake2.setVelocity(1600);
                    intake.setPower(1);
                })
                .forward(15)
                .turn(Math.toRadians(-28))
                .UNSTABLE_addTemporalMarkerOffset(2+2, ()->{
                    plateServoLeft.setPower(-0.6);
                    plateServoRight.setPower(-0.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(3+2, ()->{
                    plateServoLeft.setPower(0);
                    plateServoRight.setPower(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.8+3, ()->{
                    plateServoLeft.setPower(-0.6);
                    plateServoRight.setPower(-0.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.1+3, ()->{
                    plateServoLeft.setPower(0);
                    plateServoRight.setPower(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(6+2, ()->{
                    plateServoLeft.setPower(-0.6);
                    plateServoRight.setPower(-0.6);
                })
                .waitSeconds(200)
                .build();

        drive.setPoseEstimate(startingPose);

        limelight.start();
        timer.reset();
        waitForStart();
        thread.start();
        drive.followTrajectorySequence(DecodeAuto);
    }
}