package org.firstinspires.ftc.teamcode.TeleOPs.activitati;

import static java.lang.Math.signum;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (name = "Ziua portilor deschise")
public class teleOpOfertaEd extends LinearOpMode {
    SampleMecanumDrive drive;
    DcMotorEx intake;
    DcMotorEx outtake1;
    DcMotorEx outtake2;
    CRServo plateServoLeft;
    CRServo plateServoRight;

    ElapsedTime timer = new ElapsedTime();
    int plateTargetPosition = 0;
    double maxPlatePower = 0.5;

    DcMotorEx plateEncoder;
    boolean shootToggle = false;
    int sumPlate = 0;
    int encoderPos = 0;
    double plateError = 0;
    double platePow = 1;
    double plateP = 0.0002;
    double plateI = 0.00000;
    double plateD = 0.0008;
    int desiredPos = 0;
    boolean reverseToggle = false;
    boolean reverseOnce = false;
    boolean storeToggle = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);

        outtake2.setDirection(outtake1.getDirection());

        plateServoLeft = hardwareMap.get(CRServo.class, "plateServoLeft");
        plateServoRight = hardwareMap.get(CRServo.class, "plateServoRight");
        plateEncoder = hardwareMap.get(DcMotorEx.class, "intake");

        plateEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            if(!reverseOnce){
                intake.setPower(1);
                reverseOnce = true;
            }

            outtake1.setVelocity(1300);
            outtake2.setVelocity(1300);

            double plateDt = timer.seconds();

            encoderPos = -plateEncoder.getCurrentPosition();
            plateError = plateTargetPosition - encoderPos;
            sumPlate += (int) (plateError * plateDt);
            double deriv = (plateTargetPosition - encoderPos - plateError);

            platePow = plateP*plateError + plateI*sumPlate + deriv* plateD;

            plateTargetPosition = desiredPos;


            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            if(gamepad1.x){
                if(!shootToggle){
                    desiredPos -= 8192;
                    shootToggle = true;
                }
            } else shootToggle = false;

            if(gamepad1.b){
                if(!storeToggle){
                    desiredPos += 8192;
                    storeToggle = true;
                }
            } else storeToggle = false;

            if(gamepad1.a){
                if(!reverseToggle){
                    intake.setPower(-signum(intake.getPower()));
                    reverseToggle = true;
                }
            } else reverseToggle = false;


            if(platePow < -maxPlatePower) platePow = -maxPlatePower;
            if(platePow > maxPlatePower) platePow = maxPlatePower;

            plateServoRight.setPower(platePow);
            plateServoLeft.setPower(platePow);
        }
    }
}