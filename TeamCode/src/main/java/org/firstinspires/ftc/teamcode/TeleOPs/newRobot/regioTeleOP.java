package org.firstinspires.ftc.teamcode.TeleOPs.newRobot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "!regioTeleOP", group = "TeleOp")
public class regioTeleOP extends LinearOpMode {

    public enum StateList {
        Idle,
        Outtake
    }
    StateList state = StateList.Idle;

    private ElapsedTime timer = new ElapsedTime();

    double maxPlatePower = 0.3;

    // hardware
    DcMotorEx turret;
    Limelight3A limelight;
    SampleMecanumDrive drive;

    // hardware
    DcMotorEx plateEncoder;
    CRServo plateServoLeft;
    CRServo plateServoRight;
    DcMotorEx intake;
    DcMotorEx outtake;
    DcMotorEx outtake2;
    NormalizedColorSensor colorSensor;
    DistanceSensor distanceSensor;
    IMU imu;

    int plateTargetPosition = 0;

    int sumPlate = 0;
    int encoderPos = 0;
    double plateError = 0;

    double platePow = 1;
    double plateP = 0.0002;
    double plateI = 0.00000;
    double plateD = 0.0008;

    boolean plateAtPosition = false;
    boolean shooterReady = false;
    boolean intakeToggle = false;
    boolean manualPlateMoveLeft = false;
    boolean ballOneShot = false;
    boolean ballTwoShot = false;
    boolean ballThreeShot = false;
    boolean manualPlateMoveRight = false;
    boolean goToOuttake = false;
    int desiredPos = 0;
    double intakePower = 1;
    boolean goToIdle = false;
    boolean reverseOuttake = false;
    boolean outtakeOnce = false;
    boolean resetTimer = false;
    ElapsedTime shootCooldown = new ElapsedTime();
    boolean stopAndReset = false;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        plateServoLeft = hardwareMap.get(CRServo.class, "plateServoLeft");
        plateServoRight = hardwareMap.get(CRServo.class, "plateServoRight");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        imu = hardwareMap.get(IMU.class, "imu");

        plateEncoder = hardwareMap.get(DcMotorEx.class, "intake");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);
        plateEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO: Set your AprilTag pipeline here before each match
        // 1 - Blue alliance
        // 2 - Red alliance
        limelight.pipelineSwitch(1);
        limelight.start();



        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setTargetPosition(0);
        turret.setPower(0.1);

        shootCooldown.reset();

        while(opModeInInit() && !isStopRequested()){
            if(!stopAndReset) {
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                stopAndReset = true;
            }
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
        intake.setPower(intakePower);
        outtake2.setDirection(outtake.getDirection());

            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            plateAtPosition = plateTargetPosition < -plateEncoder.getCurrentPosition() + 400 && plateTargetPosition > -plateEncoder.getCurrentPosition() - 200;

            double plateDt = timer.seconds();

            encoderPos = -plateEncoder.getCurrentPosition();
            plateError = plateTargetPosition - encoderPos;
            sumPlate += (int) (plateError * plateDt);
            double deriv = (plateTargetPosition - encoderPos - plateError);

            platePow = plateP*plateError + plateI*sumPlate + deriv* plateD;

            plateTargetPosition = desiredPos;

            switch (state){
                case Idle:
                    ballOneShot = false;
                    ballTwoShot = false;
                    ballThreeShot = false;
                    shooterReady = false;
                    resetTimer = false;

                    if(shootCooldown.seconds() >= 5){
                        outtake.setVelocity(300);
                        outtake2.setVelocity(300);
                    }
                    break;

                case Outtake:
                    if(!resetTimer){
                        shootCooldown.reset();
                        resetTimer = true;
                    }

                    outtake.setVelocity(1300);
                    outtake2.setVelocity(1300);

                    if(outtake.getVelocity() > 990) {
                        shooterReady = true;
                    } else shooterReady = false;

                    if(shooterReady && !ballOneShot) {
                        desiredPos -= 8192/3;
                        ballOneShot = true;
                        shootCooldown.reset();
                    }

                    if(ballOneShot && !ballTwoShot && shooterReady && shootCooldown.seconds() >= 0.5){
                        desiredPos -= 8192/3;
                        ballTwoShot = true;
                        shootCooldown.reset();
                    }

                    if(ballTwoShot && !ballThreeShot && shooterReady && shootCooldown.seconds() >= 0.5){
                        desiredPos -= 8192/3;
                        ballThreeShot = true;
                    }

                    if(ballThreeShot && shooterReady){
                        state = StateList.Idle;
                        ballOneShot = false;
                        ballTwoShot = false;
                        ballThreeShot = false;
                        shooterReady = false;
                        resetTimer = false;
                    }
                    break;
            }

            if(gamepad2.leftBumperWasPressed() && !manualPlateMoveLeft){
                desiredPos -= 8129/3;
                manualPlateMoveLeft = true;
            } else manualPlateMoveLeft = false;

            if(gamepad2.rightBumperWasPressed() && !manualPlateMoveRight){
                desiredPos += 8129/3;
                manualPlateMoveRight = true;
            } else manualPlateMoveRight = false;

            if(gamepad1.dpadUpWasPressed() && !intakeToggle){
                intakePower = -intakePower;
                intakeToggle = true;
            } else intakeToggle = false;

            if(gamepad2.yWasPressed() && !goToOuttake){
                state = StateList.Outtake;
                goToOuttake = true;
            } else goToOuttake = false;

            if(gamepad2.aWasPressed() && !goToIdle){
                state = StateList.Idle;
                goToIdle = true;
            } else goToIdle = false;

            if(gamepad1.y) {
                if (!reverseOuttake) {
                    if (outtakeOnce) {
                        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
                    }
                    else {
                        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
                    }
                    outtakeOnce = !outtakeOnce;
                    reverseOuttake = true;
                }
            } else reverseOuttake = false;

            if(gamepad2.right_trigger < 0.1){
                outtake.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            if(platePow < -maxPlatePower) platePow = -maxPlatePower;
            if(platePow > maxPlatePower) platePow = maxPlatePower;

            plateServoRight.setPower(platePow);
            plateServoLeft.setPower(platePow);

            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}