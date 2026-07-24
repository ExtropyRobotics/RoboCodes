package org.firstinspires.ftc.teamcode.TeleOPs.larisa;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp (name = "movement")
public class movement extends LinearOpMode {
    // variabile
    DcMotorEx motor1;
    DcMotorEx motor2;
    Servo servus;
    int targ = 0;
    int targ2 = 0;
    double spp = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        // hwmap
        motor1 = hardwareMap.get(DcMotorEx.class, "idk.v1");
        motor2 = hardwareMap.get(DcMotorEx.class, "idk.v2");
        servus = hardwareMap.get(Servo.class, "idk.v3");

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            servus.setPosition(spp);

            if (gamepad2.dpad_up){
                spp += 0.001;
            }
            if (gamepad2.dpad_down) spp -= 0.001;

            motor1.setPower(1);

            if(gamepad1.a){
                targ += 10;
            }

            if(gamepad1.y) targ -= 10;


            motor1.setTargetPosition(targ);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            motor2.setTargetPosition(targ2);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setPower(0.1);
            if(gamepad2.a){
                targ2 += -5;
            }
            if(gamepad2.y) targ2 -= -5;

            telemetry.addData("target:", targ);
            telemetry.addData("pos:", motor1.getCurrentPosition());

            telemetry.addData("pos2:", motor2.getCurrentPosition());
            telemetry.addData("targ2:", targ2);

            telemetry.addData("targ3:", spp);

            telemetry.update();



        }

    }
}

