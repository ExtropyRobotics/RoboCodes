package org.firstinspires.ftc.teamcode.TeleOPs.Tester;

import static java.lang.Math.signum;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@TeleOp (name = "angleTst")
public class somesAngleTest extends LinearOpMode {

    // Drive
    SampleMecanumDrive drive;

    // Hardware
    DcMotorEx intake;
    DcMotorEx outtake;
    DcMotorEx plateEncoder;
    CRServo plateServoLeft;
    CRServo plateServoRight;
    Servo angle;

    // Booleans
    boolean shootToggle = false; // Used for moving plate in the shooting direction.
    boolean reverseToggle = false; // Used for reversing the intake.
    boolean storeToggle = false; // Used for moving plate in the storing direction.

    // Powers & positions
    double maxPlatePower = 0.7;
    double platePow = 1;
    double intakePower = 1;
    double diff = 0;
    int desiredPos = 0;
    double servoPoz = 0.6;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        // HardwareMap in correlation with Configuratie.txt (I hope)

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");

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

            if(gamepad1.dpad_up) servoPoz += 0.0001;
            if(gamepad1.dpad_down) servoPoz -= 0.0001;

            angle.setPosition(servoPoz);


            // == BASICS ==

            // Basic drive.
            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            // Powers intake and outtake motors for the entirety of the TeleOP.
            intake.setPower(intakePower);
            outtake.setPower(1);


            // == BUTTONS ==

            // Move the plate towards the launching direction.
            if(gamepad1.x){
                if(!shootToggle){
                    desiredPos -= 8192/3;
                    shootToggle = true;
                }
            } else shootToggle = false;

            // Move the plate towards the storing direction.
            if(gamepad1.b){
                if(!storeToggle){
                    desiredPos += 8192/3;
                    storeToggle = true;
                }
            } else storeToggle = false;

            // Reverse intake.
            if(gamepad1.a){
                if(!reverseToggle){
                    intakePower = -signum(intakePower);
                    reverseToggle = true;
                }
            } else reverseToggle = false;


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

            telemetry.addData("servo: ", servoPoz);
            telemetry.update();
        }
    }
}