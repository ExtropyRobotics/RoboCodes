package org.firstinspires.ftc.teamcode.Autonomii.CnapsysCashCup;

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

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "FarBlue")

public class FarBlue_CCC extends LinearOpMode {
    Pose2d startingPose = new Pose2d(61, -11, 0); // Starts at blue far facing blue goal

    // Hardware
    DcMotorEx intake; // intake motor
    DcMotorEx outtake; // outtake motor
    DcMotorEx outtake2;
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
    double farVelocity = 1350; // optimal velocity for shooting from afar
    double closeVelocity = 1250; // optimal velocity for shooting from close range (we don't need this here)


    class autoThread implements Runnable { // Using thread to add a new while
        @Override
        public void run() {

            while (opModeIsActive() && !isStopRequested()) { // While runs through the entirety of the autonomous.

                // Powers intake and outtake motors for the entirety of the Autonomous.
                intake.setPower(intakePower);
                outtake.setVelocity(motorVelocity);
                outtake2.setVelocity(motorVelocity);

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
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake");

        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);

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
                    motorVelocity = farVelocity - 50;
                }) // 0.1

                // == PRELOAD (1) ==

                // Spline to shoot preload
                .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(new Pose2d(50, -16, Math.toRadians(19)), Math.toRadians(-155))

                // Shoot first ball
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    desiredPos -= 8192/3;
                }) // 1.1

                // Shoot second ball and increase velo to compensate RPM lost by friction
                .UNSTABLE_addTemporalMarkerOffset(1.7, ()->{
                    desiredPos -= 8192/3;
                    motorVelocity = farVelocity + 50;
                }) // 1.2

                // Shoot third ball
                .UNSTABLE_addTemporalMarkerOffset(3, ()->{
                    desiredPos -= 8192/3;
                }) // 1.3

                // Give robot time to shoot
                .setTangent(Math.toRadians(-70))
                .waitSeconds(3.4)

                // == HUMAN PLAYER (2) ==

                // Spline to human player (takes right-most artefact)
                .splineToSplineHeading(new Pose2d(55, -35.5, Math.toRadians(-80)), Math.toRadians(-70))
                .splineToConstantHeading(new Vector2d(58, -57), Math.toRadians(-90))
                .setVelConstraint(new TranslationalVelocityConstraint(9))
                .splineToConstantHeading(new Vector2d(58, -60), Math.toRadians(-90))

                // Store first ball and ready velo up for shooting
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
                    motorVelocity = farVelocity + 100;
                    desiredPos += 8192/3;
                }) // 2.1

                // Third ball is NOT worth it </3

                // Spline to 2nd ball
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(61, -55), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(64, -60), Math.toRadians(-90))
                .resetVelConstraint()

                // Spline to shooting position
                .setTangent(Math.toRadians(105))
                .splineToConstantHeading(new Vector2d(59, -38), Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(55, -16, Math.toRadians(16)), Math.toRadians(100))

                // Shoot first ball, decrease RPM to fix constant overshoot
                .UNSTABLE_addTemporalMarkerOffset(0.25, ()->{
                    desiredPos -= 8192/3;
                    motorVelocity = farVelocity;
                }) // 2.2

                // Shoot second ball
                .UNSTABLE_addTemporalMarkerOffset(1.55, ()->{
                    desiredPos -= 8192/3;
                }) // 2.3

                .setTangent(Math.toRadians(-160))
                .waitSeconds(1.9) // give robot time to shoot

                // == THIRD SET (3) ==

                // Spline to set #3
                .setTangent(Math.toRadians(-160))
                .splineToSplineHeading(new Pose2d(39, -20, Math.toRadians(-90)), Math.toRadians(-160))
                .splineToConstantHeading(new Vector2d(34, -22), Math.toRadians(-90))
                .setVelConstraint(new TranslationalVelocityConstraint(11))
                .splineToConstantHeading(new Vector2d(34, -53), Math.toRadians(-90))
                .resetVelConstraint()
                .splineToSplineHeading(new Pose2d(37, -63, Math.toRadians(-90)), Math.toRadians(65))

                // Store first ball
                .UNSTABLE_addTemporalMarkerOffset(-2.4, ()->{
                    desiredPos += 8192/3;
                }) // 3.1

                // Store second ball and increase velo to compensate for rpm lost by friction
                .UNSTABLE_addTemporalMarkerOffset(-1.65, ()->{
                    desiredPos += 8192/3;
                    motorVelocity = farVelocity + 100;
                }) // 3.2

                // Spline to shoot
                .splineToSplineHeading(new Pose2d(55, -16, Math.toRadians(12)), Math.toRadians(65))

                // Shoot first ball
                .UNSTABLE_addTemporalMarkerOffset(0.25, ()->{
                    desiredPos -= 8192/3;
                }) // 3.3

                // Shoot second ball
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
                    desiredPos -= 8192/3;
                }) // 3.4

                // Shoot third ball
                .UNSTABLE_addTemporalMarkerOffset(2.75, ()->{
                    desiredPos -= 8192/3;
                }) // 3.5

                .waitSeconds(100)

                .build();

        drive.setPoseEstimate(startingPose);

        waitForStart();
        thread.start();
        drive.followTrajectorySequence(autonomous);
    }
}