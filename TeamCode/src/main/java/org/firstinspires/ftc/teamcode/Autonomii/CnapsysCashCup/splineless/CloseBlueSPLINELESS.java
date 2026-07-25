package org.firstinspires.ftc.teamcode.Autonomii.CnapsysCashCup.splineless;

import static java.lang.Math.signum;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "CloseBlueSPLINELESS")

public class CloseBlueSPLINELESS extends LinearOpMode {
    Pose2d startingPose = new Pose2d(-50, -48, Math.toRadians(60)); // Starts at blue goal facing blue goal

    // Hardware
    DcMotorEx intake; // intake motor
    DcMotorEx outtake; // outtake motor
    DcMotorEx outtake2;
    DcMotorEx plateEncoder; // separate encoder for plate (8192 ticks through bore rev encoder)
    CRServo plateServoLeft; // continuous so it's not stuck between 0-1 values
    CRServo plateServoRight; // continuous so it's not stuck between 0-1 values

    // Powers & positions
    int fullPlateRotation = 8192; // Ticks encoder recognizes for 1 full rotation
    int desiredPos = 0; // ideal plate position
    int plateTolerance = 500;
    double maxPlatePower = 1;
    double platePow = 1; // calculated plate power (-maxPlatePower or maxPlatePower)
    double intakePower = 1; // is reversed by X button
    double diff = 0; // used for calculating difference between ideal plate position and real plate position
    double motorVelocity = 1300; // changes depending on driver input
    double farVelocity = 2200; // optimal velocity for shooting from afar
    double closeVelocity = 850+70; // optimal velocity for shooting from close range
    boolean pulete; // pulete


    class autoThread implements Runnable { // Using thread to add a new while
        @Override
        public void run() {

            while (opModeIsActive() && !isStopRequested()) { // While runs through the entirety of the autonomous.

                // Powers intake and outtake motors for the entirety of the Autonomous.
                intake.setPower(intakePower);
                outtake.setVelocity(motorVelocity);
                outtake2.setVelocity(motorVelocity);

                // == PLATE CALCULATIONS ==

                // Calculates the difference between the target position and the actual position.
                diff = desiredPos - plateEncoder.getCurrentPosition();

            /* If the calculated difference is higher than 500 ticks (plateTolerance value),
            moves the plate in the corresponding direction making the difference as small as possible.
            The 500 ticks tolerance is absolutely necessary to reduce oscillations. */

                if(plateEncoder.getCurrentPosition() > desiredPos - (plateTolerance/2) && plateEncoder.getCurrentPosition() < desiredPos + (plateTolerance/2)){
                    platePow = 0;
                } else platePow = signum(diff) * maxPlatePower;

                // Power the plate servos.
                plateServoRight.setPower(platePow);
                plateServoLeft.setPower(platePow);
            }
        }
    }

    autoThread obj = new autoThread();

    @Override
    public void runOpMode(){
        Thread thread = new Thread(obj);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // HardwareMap in correlation with Configuratie.txt (I hope)

        outtake = hardwareMap.get(DcMotorEx.class, "outtake2");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake");

        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);

        plateServoLeft = hardwareMap.get(CRServo.class, "plateServoLeft");
        plateServoRight = hardwareMap.get(CRServo.class, "plateServoRight");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        plateEncoder = hardwareMap.get(DcMotorEx.class, "plateEncoder");
        plateEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plateEncoder.setDirection(DcMotorSimple.Direction.REVERSE);



        TrajectorySequence autonomous = drive.trajectorySequenceBuilder(startingPose)

                // Start outtake motor
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    motorVelocity = closeVelocity;
                })

                .waitSeconds(2)

                // == PRELOAD (1) ==

                // Give robot time to shoot
                .waitSeconds(0.9)

                // Shoot first ball (increase velo to compensate RPM lost by friction)
                .UNSTABLE_addTemporalMarkerOffset(-0.9, ()->{
                    desiredPos -= 8192/3;
                    motorVelocity = closeVelocity + 200;
                }) // 1.1
                // Shoot second ball
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->{
                    desiredPos -= 8192/3;
                }) // 1.2

                // Shoot third ball
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->{
                    desiredPos -= 8192/3;
                }) // 1.3


                // == FIRST SET (2) ==
                .waitSeconds(3)

                // Store first ball
                .UNSTABLE_addTemporalMarkerOffset(-1.8, ()->{
                    desiredPos += 8192/3;
                    motorVelocity = closeVelocity;
                }) // 2.1
//
//                // Store second ball
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    desiredPos += 8192/3;
                }) // 2.2
//
//                // (Third ball comes in without needing another rotate)
//
                .waitSeconds(4)
//
//                // Give robot time to shoot
                .waitSeconds(0.9)
//
                .UNSTABLE_addTemporalMarkerOffset(-0.9, ()->{
                    desiredPos -= 8192/3;
                    motorVelocity = closeVelocity + 200;
                }) // 1.1
                // Shoot second ball
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->{
                    desiredPos -= 8192/3;
                }) // 1.2

                // Shoot third ball
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->{
                    desiredPos -= 8192/3;
                }) // 1.3
//
//                // == SECOND SET (3) ==
                .waitSeconds(4)
//
//                // Store first ball
                .UNSTABLE_addTemporalMarkerOffset(-2.7, ()->{
                    desiredPos += 8192/3;
                    motorVelocity = closeVelocity;
                }) // 3.1
//
//                // Store second ball
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    desiredPos += 8192/3;
                }) // 3.2
//
//                // (Third ball comes in without needing another rotate)
//
                .waitSeconds(4)
//
//                // Shoot first ball
                .UNSTABLE_addTemporalMarkerOffset(-0.9, ()->{
                    desiredPos -= 8192/3;
                    motorVelocity = closeVelocity + 200;
                }) // 1.1
                // Shoot second ball
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->{
                    desiredPos -= 8192/3;
                }) // 1.2

                // Shoot third ball
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->{
                    desiredPos -= 8192/3;
                }) // 1.3
//                // Give robot time to shoot
                .waitSeconds(1)
//
//                // == THIRD SET (4) ==
                .waitSeconds(4)
//
//                // Store first ball
                .UNSTABLE_addTemporalMarkerOffset(-2.2, ()->{
                    desiredPos += 8192/3;
                    motorVelocity = closeVelocity ;
                }) // 4.1
//
//                // Store second ball
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    desiredPos += 8192/3;
                    motorVelocity = closeVelocity + 50;
                }) // 4.2
//
                .waitSeconds(5)
//
//                // Shoot first ball
                .UNSTABLE_addTemporalMarkerOffset(-0.9, ()->{
                    desiredPos -= 8192/3;
                    motorVelocity = closeVelocity + 200;
                }) // 1.1
                // Shoot second ball
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->{
                    desiredPos -= 8192/3;
                }) // 1.2

                // Shoot third ball
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->{
                    desiredPos -= 8192/3;
                }) // 1.3

                .waitSeconds(100)

                .build();

        drive.setPoseEstimate(startingPose);

        waitForStart();
        thread.start();
        drive.followTrajectorySequence(autonomous);
    }
}