package org.firstinspires.ftc.teamcode.TeleOPs.oldRobot.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp (name = "targetPoz")
public class targetPoz extends LinearOpMode {

    private DcMotor plate;
    private int plateTargetPoz = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        plate = hardwareMap.get(DcMotor.class, "plate");
        plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        plate.setTargetPosition(plateTargetPoz);

        waitForStart();

        while(opModeIsActive()) {

        }
    }
}
