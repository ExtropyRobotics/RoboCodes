package org.firstinspires.ftc.teamcode.TeleOPs.oldRobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@TeleOp(name = "TeleOP")
public class TeleOP extends LinearOpMode {
    private DcMotor launcher;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        launcher = hardwareMap.get(DcMotor.class, "plate");

        waitForStart();

        while(opModeIsActive()){

            if(gamepad2.a){
                launcher.setPower(1);
            } else launcher.setPower(0);
        }
    }
}
