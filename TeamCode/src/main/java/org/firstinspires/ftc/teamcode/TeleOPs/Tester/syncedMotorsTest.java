package org.firstinspires.ftc.teamcode.TeleOPs.Tester;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp (name = "sync", group = "Tester")
public class syncedMotorsTest extends LinearOpMode {
    DcMotorEx motor1;
    DcMotorEx motor2;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        waitForStart();

        while(opModeIsActive()){



            telemetry.addData("RPM2:", motor2.getVelocity());
            telemetry.addData("RPM1:", motor1.getVelocity());
            telemetry.update();
        }
    }
}