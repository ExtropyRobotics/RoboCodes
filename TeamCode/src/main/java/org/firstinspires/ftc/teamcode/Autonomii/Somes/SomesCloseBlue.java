package org.firstinspires.ftc.teamcode.Autonomii.Somes;

import static java.lang.Math.signum;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "CloseBlue")

public class SomesCloseBlue extends LinearOpMode {
    Pose2d startingPose = new Pose2d(-50, -48, Math.toRadians(60)); // Starts at blue goal facing blue goal.

    // Hardware
    DcMotorEx intake; // intake motor
    DcMotorEx outtake; // outtake motor
    DcMotorEx plateEncoder; // separate encoder for plate (8192 ticks through bore rev encoder)
    CRServo plateServoLeft; // continuous so it's not stuck between 0-1 values
    CRServo plateServoRight; // continuous so it's not stuck between 0-1 values
    Servo angle; // ramp servo

    // Powers & positions
    int desiredPos = 0; // ideal plate position
    double maxPlatePower = 0.7;
    double platePow = 1; // calculated plate power (-0.7 or 0.7)
    double intakePower = 1; // is reversed by X button
    double diff = 0; // used for calculating difference between ideal plate position and real plate position
    double servoPoz = 0.63; // constant for both close and far
    double motorVelocity = 1250; // changes depending on driver input
    double farVelocity = 1700; // optimal velocity for shooting from afar
    double closeVelocity = 1250; // optimal velocity

    class autoThread implements Runnable { // Using thread to add a new while
        @Override
        public void run() {

            while (opModeIsActive() && !isStopRequested()) { // While runs through the entirety of the autonomous.

                // Locks servo in position.
                angle.setPosition(servoPoz);

                // Powers intake and outtake motors for the entirety of the Autonomous.
                intake.setPower(intakePower);
                outtake.setVelocity(motorVelocity);

                // Calculates the difference between the target position and the actual position.
                diff = desiredPos - plateEncoder.getCurrentPosition();

                /* If the calculated difference is higher than 500 ticks (the sum of 250 and 250 from the formula)
                moves the plate in the corresponding direction making the difference as small as possible.
                The 500 ticks tolerance is absolutely necessary to reduce oscillations. */

                if(plateEncoder.getCurrentPosition() > desiredPos - 250 && plateEncoder.getCurrentPosition() < desiredPos + 250){
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

        outtake.setDirection(DcMotorSimple.Direction.REVERSE);

        angle = hardwareMap.get(Servo.class, "angle");

        plateServoLeft = hardwareMap.get(CRServo.class, "plateServoLeft");
        plateServoRight = hardwareMap.get(CRServo.class, "plateServoRight");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        plateEncoder = hardwareMap.get(DcMotorEx.class, "plateEncoder");
        plateEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plateEncoder.setDirection(DcMotorSimple.Direction.REVERSE);


        TrajectorySequence autonomous = drive.trajectorySequenceBuilder(startingPose)

                // Spline to first shooting position
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    outtake.setVelocity(closeVelocity);
                })
                .setTangent(Math.toRadians(38))
                .splineToSplineHeading(new Pose2d(-17, -22, Math.toRadians(45)), Math.toRadians(38))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{
                    desiredPos -= 8192/3;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, ()->{
                    desiredPos -= 8192/3;
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, ()->{
                    desiredPos -= 8192/3;
                })
                .waitSeconds(100)
                //.setVelConstraint(new TranslationalVelocityConstraint(0))
                //.setTangent(Math.toRadians(0))
                //.splineToSplineHeading(new Pose2d(0, 0, Math.toRadians(-90)), Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(-90))

                //.UNSTABLE_addTemporalMarkerOffset(0.5, ()->{

                // Unstable contents here

                //})

                .build();

        drive.setPoseEstimate(startingPose);

        waitForStart();
        thread.start();
        drive.followTrajectorySequence(autonomous);
    }
}