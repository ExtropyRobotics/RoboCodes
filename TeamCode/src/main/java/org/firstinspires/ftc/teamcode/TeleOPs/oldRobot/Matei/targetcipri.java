package org.firstinspires.ftc.teamcode.TeleOPs.oldRobot.Matei;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp (name = "targetcrow")
public class targetcipri extends LinearOpMode {

    private DcMotor plate;
    private DcMotor farfurie;
    private int plateTargetposi = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        plate = hardwareMap.get(DcMotor.class, "targetcrow");
        initMotor(plate);
        initMotor(farfurie);

        while (opModeIsActive()) {
            if (gamepad2.a && plateTargetposi < 800)
                plateTargetposi += 100;
            plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    private void initMotor(DcMotor motor){
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}

