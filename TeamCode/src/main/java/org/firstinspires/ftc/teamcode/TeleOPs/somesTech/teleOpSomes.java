package org.firstinspires.ftc.teamcode.TeleOPs.somesTech;

import static java.lang.Math.signum;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (name = "teleOP SOMES")
public class teleOpSomes extends LinearOpMode {

    // Drive
    SampleMecanumDrive drive; // basic drive

    // Hardware
    DcMotorEx intake; // intake motor
    DcMotorEx outtake; // outtake motor
    DcMotorEx plateEncoder; // separate encoder for plate (8192 ticks through bore rev encoder)
    CRServo plateServoLeft; // continuous so it's not stuck between 0-1 values
    CRServo plateServoRight; // continuous so it's not stuck between 0-1 values
    Servo angle; // ramp servo

    // Booleans
    boolean shootToggle = false; // Used for moving plate in the shooting direction.
    boolean reverseToggle = false; // Used for reversing the intake.
    boolean storeToggle = false; // Used for moving plate in the storing direction.
    boolean farToggle =  false; // Used for switching outtake motor velocity.
    boolean closeToggle =  false; // Used for switching outtake motor velocity.

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


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

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

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            // == BASICS ==

            // Basic drive.
            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            // Locks servo in position.
            angle.setPosition(servoPoz);

            // Powers intake and outtake motors for the entirety of the TeleOP.
            intake.setPower(intakePower);
            outtake.setVelocity(motorVelocity);


            // == BUTTONS ==

            // Move the plate towards the launching direction.
            if(gamepad2.x){
                if(!shootToggle){
                    desiredPos -= 8192/3;
                    shootToggle = true;
                }
            } else shootToggle = false;

            // Move the plate towards the storing direction.
            if(gamepad2.b){
                if(!storeToggle){
                    desiredPos += 8192/3;
                    storeToggle = true;
                }
            } else storeToggle = false;

            // Reverse intake.
            if(gamepad2.a){
                if(!reverseToggle){
                    intakePower = -signum(intakePower);
                    reverseToggle = true;
                }
            } else reverseToggle = false;

            // Change outtake motor velocity to launch from afar.
            if(gamepad2.dpad_up){
                if(!farToggle){
                    motorVelocity = farVelocity;
                    farToggle = true;
                }
            } else farToggle = false;

            // Change outtake motor velocity to launch close.
            if(gamepad2.dpad_down){
                if(!closeToggle){
                    motorVelocity = closeVelocity;
                    closeToggle = true;
                }
            } else closeToggle = false;

            // == PLATE CALCULATIONS ==

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