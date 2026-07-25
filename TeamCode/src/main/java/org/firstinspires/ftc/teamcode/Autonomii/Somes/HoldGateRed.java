package org.firstinspires.ftc.teamcode.Autonomii.Somes;

import static java.lang.Math.signum;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
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

//@Disabled
@Autonomous (name = "HoldGateRed")

public class HoldGateRed extends LinearOpMode {
    Pose2d startingPose = new Pose2d(-50, 48, Math.toRadians(-60)); // Starts at blue goal facing blue goal

    // Hardware
    DcMotorEx intake; // intake motor
    DcMotorEx outtake; // outtake motor
    DcMotorEx plateEncoder; // separate encoder for plate (8192 ticks through bore rev encoder)
    CRServo plateServoLeft; // continuous so it's not stuck between 0-1 values
    CRServo plateServoRight; // continuous so it's not stuck between 0-1 values

    // Powers & positions
    int desiredPos = 0; // ideal plate position
    int plateTolerance = 650;
    double maxPlatePower = 1; // best power for plate
    double platePow = 1; // calculated plate power (-0.7 or 0.7)
    double intakePower = 1; // intake runs at max power
    double diff = 0; // used for calculating difference between ideal plate position and real plate position
    double motorVelocity = 1250;
    double farVelocity = 1700; // optimal velocity for shooting from afar (we don't need this here)
    double closeVelocity = 1250; // optimal velocity for shooting from close range


    class autoThread implements Runnable { // Using thread to add a new while
        @Override
        public void run() {

            while (opModeIsActive() && !isStopRequested()) { // While runs through the entirety of the autonomous.

                // Powers intake and outtake motors for the entirety of the Autonomous.
                intake.setPower(intakePower);
                outtake.setVelocity(motorVelocity);

                // Calculates the difference between the target position and the actual position.
                diff = desiredPos - plateEncoder.getCurrentPosition();

                /* If the calculated difference is higher than 500 ticks (the sum of 275 and 275 from the formula)
                moves the plate in the corresponding direction making the difference as small as possible.
                The 500 ticks tolerance is absolutely necessary to reduce oscillations. */

                if(plateEncoder.getCurrentPosition() > desiredPos - (plateTolerance / 2) && plateEncoder.getCurrentPosition() < desiredPos + (plateTolerance / 2)){
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

        plateServoLeft = hardwareMap.get(CRServo.class, "plateServoLeft");
        plateServoRight = hardwareMap.get(CRServo.class, "plateServoRight");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        plateEncoder = hardwareMap.get(DcMotorEx.class, "plateEncoder");
        plateEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plateEncoder.setDirection(DcMotorSimple.Direction.REVERSE);


        TrajectorySequence autonomous = drive.trajectorySequenceBuilder(startingPose)

                // Start outtake motor
                .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                    motorVelocity = closeVelocity - 200;
                }) // 0.1

                // == PRELOAD (1) ==

                // Spline to first shooting position
                .setTangent(Math.toRadians(-45))
                .splineToSplineHeading(new Pose2d(-18, 18, Math.toRadians(-55)), Math.toRadians(-45))

                // Give robot time to shoot
                .waitSeconds(1)

                // Shoot first ball (increase velo to compensate RPM lost by friction)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, ()->{
                    desiredPos -= 8192/3;
                    motorVelocity = closeVelocity;
                }) // 1.1

                // Shoot second ball
                .UNSTABLE_addTemporalMarkerOffset(-0.7, ()->{
                    desiredPos -= 8192/3;
                    motorVelocity = closeVelocity + 50;
                }) // 1.2

                // Shoot third ball
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{
                    desiredPos -= 8192/3;
                }) // 1.3

                // == FIRST SET (2) ==

                // Spline to the first set
                .splineToSplineHeading(new Pose2d(-13, 17, Math.toRadians(90)), Math.toRadians(90))
                .setVelConstraint(new TranslationalVelocityConstraint(13))
                .splineToConstantHeading(new Vector2d(-10, 30), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-10, 45), Math.toRadians(90))
                .resetVelConstraint()
                .splineToConstantHeading(new Vector2d(-13, 54), Math.toRadians(-90))

                // Store first ball
                .UNSTABLE_addTemporalMarkerOffset(-1.8, ()->{
                    desiredPos += 8192/3;
                }) // 2.1

                // Store second ball
                .UNSTABLE_addTemporalMarkerOffset(-1.15, ()->{
                    desiredPos += 8192/3;
                }) // 2.2

                // (Third ball comes in without needing another rotate)

                // Spline to shooting position.
                .splineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(-55)), Math.toRadians(-110))

                // Give robot time to shoot
                .waitSeconds(1.1)

                // Shoot first ball
                .UNSTABLE_addTemporalMarkerOffset(-1.3, ()->{
                    desiredPos -= 8192/3;
                }) // 2.3

                // Shoot second ball
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()->{
                    desiredPos -= 8192/3;
                }) // 2.4

                // Shoot third ball
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->{
                    desiredPos -= 8192/3;
                }) // 2.5

                // == SECOND SET (3) ==

                // Spline to second set
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(11, 15, Math.toRadians(90)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(15, 15), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(18, 16), Math.toRadians(90))
                .setVelConstraint(new TranslationalVelocityConstraint(13))
                .splineToConstantHeading(new Vector2d(20, 53), Math.toRadians(90))
                .resetVelConstraint()
                .splineToSplineHeading(new Pose2d(20, 62, Math.toRadians(90)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(20, 50), Math.toRadians(-90))

                // Store first ball
                .UNSTABLE_addTemporalMarkerOffset(-3.2, ()->{
                    desiredPos += 8192/3;
                }) // 3.1

                // Store second ball
                .UNSTABLE_addTemporalMarkerOffset(-2.3, ()->{
                    desiredPos += 8192/3;
                }) // 3.2

                // (Third ball comes in without needing another rotate)

                // Spline to shooting position
                .splineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(-48)), Math.toRadians(-125))

                // Shoot first ball
                .UNSTABLE_addTemporalMarkerOffset(-0.8, ()->{
                    desiredPos -= 8192/3;
                    motorVelocity = closeVelocity + 50;
                }) // 3.3

                // Shoot second ball
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->{
                    desiredPos -= 8192/3;
                }) // 3.4

                // Shoot third ball
                .UNSTABLE_addTemporalMarkerOffset(0.15, ()->{
                    desiredPos -= 8192/3;
                }) // 3.5

                // Give robot time to shoot
                .setTangent(Math.toRadians(0))
                .waitSeconds(1)

                // Spline to open gate
                .splineToSplineHeading(new Pose2d(2, 48, Math.toRadians(0)), Math.toRadians(90))
                .waitSeconds(1)

                .waitSeconds(1)

                .build();

        drive.setPoseEstimate(startingPose);

        waitForStart();
        thread.start();
        drive.followTrajectorySequence(autonomous);
    }
}