package org.firstinspires.ftc.teamcode.TeleOPs.Tester;

import static java.lang.Math.signum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (name = "MotorTester", group = "Tester")
public class motorTester extends LinearOpMode {
    boolean reverseToggle = false;
    boolean powerUp = false;
    boolean powerDown = false;
    DcMotorEx motor;
    double power = 1;


    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotorEx.class, "motor");

        waitForStart();

        while(opModeIsActive()){

            motor.setPower(power);

            if(gamepad1.a){
                if(!reverseToggle){
                    power = -signum(power)*power;
                    reverseToggle = true;
                }
            } else reverseToggle = false;

            if(gamepad1.dpad_up){
                if(!powerUp){
                    power += 0.1;
                    powerUp = true;
                }
            } else powerUp = false;

            if(gamepad1.dpad_down){
                if(!powerDown){
                    power += 0.1;
                    powerDown = true;
                }
            } else powerDown = false;

            telemetry.addData("RPM", motor.getVelocity());
            telemetry.addData("Power", power);
            telemetry.addLine(" ");
            telemetry.addLine("X to inverse direction");
            telemetry.addLine("dpad down to decrease power");
            telemetry.addLine("dpad up to increase power");

            telemetry.update();
        }
    }
}