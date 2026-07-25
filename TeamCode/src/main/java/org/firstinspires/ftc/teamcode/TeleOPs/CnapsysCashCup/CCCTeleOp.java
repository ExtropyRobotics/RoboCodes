package org.firstinspires.ftc.teamcode.TeleOPs.CnapsysCashCup;

import static java.lang.Math.signum;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (name = "TeleOP CCC (DUO)")
    public class CCCTeleOp extends LinearOpMode {

    // State list class for state machine
    public enum StateList{
        Idle,
        Outtake,
    }
    StateList currentState = StateList.Idle; // State changes on driver input
    ElapsedTime shootingTimer = new ElapsedTime(); // Timer used by state machine

    // Drive
    SampleMecanumDrive drive; // basic drive

    // Hardware
    DcMotorEx intake; // intake motor
    DcMotorEx outtake; // outtake motor
    DcMotorEx outtake2; // another outtake motor
    DcMotorEx plateEncoder; // separate encoder for plate (8192 ticks through bore rev encoder)
    CRServo plateServoLeft; // continuous so it's not stuck between 0-1 values
    CRServo plateServoRight; // continuous so it's not stuck between 0-1 values

    // Booleans
    boolean shootToggle = false; // Used for moving plate in the shooting direction.
    boolean reverseToggle = false; // Used for reversing the intake.
    boolean storeToggle = false; // Used for moving plate in the storing direction.
    boolean farToggle =  false; // Used for switching outtake motor velocity.
    boolean closeToggle =  false; // Used for switching outtake motor velocity.
    boolean stateMachineToggle = false; // Used for state machine button
    boolean pulete; // pulete

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
    double closeVelocity = 920; // optimal velocity for shooting from close range

    // State machine logic & values
    double previousVelocity = 0; // stored velocity value before state machine goes into outtake state
    boolean firstArtefactToggle = false;
    boolean remainingArtefactsToggle = false;
    boolean increaseVeloToggle = false;

    /* These booleans make it so the if statements only happen once,
    preventing mishaps throughout the entirety of the outtake state */

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

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

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            // == BASICS ==

            // Basic drive.
            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            // Powers intake and outtake motors for the entirety of the TeleOP.
            intake.setPower(intakePower);
            outtake.setVelocity(motorVelocity);
            outtake2.setVelocity(motorVelocity);


            // == BUTTONS ==

            // Move the plate towards the launching direction.
            if(gamepad2.x){
                if(!shootToggle){
                    desiredPos -= fullPlateRotation/3;
                    shootToggle = true;
                }
            } else shootToggle = false;

            // Move the plate towards the storing direction.
            if(gamepad2.b){
                if(!storeToggle){
                    desiredPos += fullPlateRotation/3;
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

            // Change outtake motor velocity to launch from close range.
            if(gamepad2.dpad_down){
                if(!closeToggle){
                    motorVelocity = closeVelocity;
                    closeToggle = true;
                }
            } else closeToggle = false;

            // Run state machine, launching 3 artefacts in rapid succession
            if(gamepad2.y){
                if(!stateMachineToggle){
                    currentState = StateList.Outtake;
                    stateMachineToggle = true;
                }
            } else stateMachineToggle = false;


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


            // == STATE MACHINE ==

            /* Using state machine to increase RPM precisely while shooting, decreasing time wasted in TeleOP.
            This method is an alternative to regular timed shooting */

            // !! This might not work very well if battery voltage is low
            // !! Only works if the robot has 3 artefacts, any less and it will severely overshoot

            switch (currentState){

                case Idle:

                    shootingTimer.reset(); // Keeps timer at 0 while idle
                    previousVelocity = motorVelocity; // Storing velocity value before outtake state happens

                    // Keeps toggles false while idle
                    firstArtefactToggle = false;
                    increaseVeloToggle = false;
                    remainingArtefactsToggle = false;

                    break;

                case Outtake:

                    if(shootingTimer.seconds() >= 0 && !firstArtefactToggle){
                        motorVelocity = previousVelocity + 140;
                        desiredPos -= fullPlateRotation;
                        firstArtefactToggle = true;
                    }

                    if(shootingTimer.seconds() >= 1.2){
                        currentState = StateList.Idle;
                    }

                    break;
            }

        }
    }
}