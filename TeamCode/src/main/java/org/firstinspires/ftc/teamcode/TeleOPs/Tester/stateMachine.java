package org.firstinspires.ftc.teamcode.TeleOPs.Tester;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "stateMachine")
public class stateMachine extends LinearOpMode {

    public enum states {
        IDLE,
        ROTATE
    }

    ElapsedTime timer = new ElapsedTime();
    states state = states.IDLE;
    DcMotor motor;
    boolean toggle = false;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotor.class, "servo");
        timer.reset();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            switch(state){

                case IDLE:

                    motor.setPower(0);
                    break;

                case ROTATE:

                    motor.setPower(1);
                    if(timer.seconds() > 3) state = states.IDLE;
                    break;
            }

            if(gamepad1.y && !toggle){
                state = states.ROTATE;
                timer.reset();
                toggle = true;
            } else toggle = false;

            telemetry.addData("state:", state);
            telemetry.addData("timer", timer.seconds());
            telemetry.update();
        }
    }
}