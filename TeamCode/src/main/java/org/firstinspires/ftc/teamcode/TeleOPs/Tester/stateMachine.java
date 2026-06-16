package org.firstinspires.ftc.teamcode.TeleOPs.Tester;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp (name = "stateMachine")
public class stateMachine extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();
    CRServo plateServoLeft;
    CRServo plateServoRight;
    DcMotorEx plateEncoder;
    boolean toggle = false;
    int plateTargetPosition = 0;
    double maxPlatePower = 1;

    int sumPlate = 0;
    int encoderPos = 0;
    double plateError = 0;

    double platePow = 1;
    double plateP = 0.0002;
    double plateI = 0.00000;
    double plateD = 0.0008;
    int desiredPos = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        plateServoLeft = hardwareMap.get(CRServo.class, "plateServoLeft");
        plateServoRight = hardwareMap.get(CRServo.class, "plateServoRight");
        plateEncoder = hardwareMap.get(DcMotorEx.class, "intake");
        plateEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        timer.reset();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            double plateDt = timer.seconds();

            encoderPos = -plateEncoder.getCurrentPosition();
            plateError = plateTargetPosition - encoderPos;
            sumPlate += (int) (plateError * plateDt);
            double deriv = (plateTargetPosition - encoderPos - plateError);

            platePow = plateP*plateError + plateI*sumPlate + deriv* plateD;

            plateTargetPosition = desiredPos;

            if(gamepad1.a){
                if(!toggle){
                    desiredPos -= 8192*2/3;

                    toggle = true;
                }
            } else toggle = false;

            if(platePow < -maxPlatePower) platePow = -maxPlatePower;
            if(platePow > maxPlatePower) platePow = maxPlatePower;

            plateServoRight.setPower(platePow);
            plateServoLeft.setPower(platePow);

            telemetry.addData("pos:", desiredPos);
            telemetry.update();
        }
    }
}