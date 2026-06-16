package org.firstinspires.ftc.teamcode.TeleOPs.somesTech;

import static java.lang.Math.signum;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (name = "TeleOpSomes")
public class teleOpSomes extends LinearOpMode {

    // Drive
    SampleMecanumDrive drive;

    // Hardware
    DcMotorEx intake;
    DcMotorEx outtake;
    DcMotorEx turret;
    DcMotorEx plateEncoder;
    CRServo plateServoLeft;
    CRServo plateServoRight;

    // Booleans
    boolean shootToggle = false; // Used for moving plate in the shooting direction.
    boolean reverseToggle = false; // Used for reversing the intake.
    boolean storeToggle = false; // Used for moving plate in the storing direction.
    boolean reverseOnce = false; // Reverses intake ONCE at the start of teleOP.

    // Powers & positions
    double maxPlatePower = 0.7;
    double platePow = 1;
    int desiredPos = 0;
    double diff = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // HardwareMap in correlation with Configuratie.txt (I hope)

        drive = new SampleMecanumDrive(hardwareMap);

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);

        plateServoLeft = hardwareMap.get(CRServo.class, "plateServoLeft");
        plateServoRight = hardwareMap.get(CRServo.class, "plateServoRight");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        plateEncoder = hardwareMap.get(DcMotorEx.class, "plateEncoder");
        plateEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plateEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            // Keep turret from moving. (placeholder (maybe))
            turret.setTargetPosition(0);
            turret.setPower(0.1);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Sets intake power to 1 at the start of teleOP.
            if(!reverseOnce){
                intake.setPower(1);
                reverseOnce = true;
            }

            // Outtake motor runs through the entirety of the teleOP.
            outtake.setVelocity(1200);

            // Basic drive.
            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

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

            // Reverse intake direction. (in case an artefact gets stuck)
            if(gamepad1.a){
                if(!reverseToggle){
                    intake.setPower(-signum(intake.getPower()));
                    reverseToggle = true;
                }
            } else reverseToggle = false;

            // Calculates the difference between the target position and the actual position.
            diff = desiredPos - plateEncoder.getCurrentPosition();

            /* If the calculated difference is higher than 500 ticks (the sum of 250 and 250 from the formula),
            moves the plate in the corresponding direction making the difference as small as possible.
            The 500 ticks tolerance is absolutely needed to reduce oscillations. */

            if(plateEncoder.getCurrentPosition() > desiredPos - 250 && plateEncoder.getCurrentPosition() < desiredPos + 250){
                platePow = 0;
            } else platePow = signum(diff) * maxPlatePower;

            // Power the plate servos.
            plateServoRight.setPower(platePow);
            plateServoLeft.setPower(platePow);
        }
    }
}