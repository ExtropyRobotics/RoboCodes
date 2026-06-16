package org.firstinspires.ftc.teamcode.TeleOPs.activitati;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@TeleOp (name = "Activitate Petrana")
public class teleOpPetrana extends LinearOpMode {
    SampleMecanumDrive drive;
    DcMotorEx turret;
    int turretTarget = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(turretTarget);
        turret.setPower(1);

        while(opModeInInit() && !isStopRequested()){
            telemetry.addLine("Make sure turret is centered!!");
            telemetry.addLine("Also ignore I2C warning");
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            turret.setTargetPosition(turretTarget);

            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            if(gamepad1.dpad_right) turretTarget -= 1;
            if(gamepad1.dpad_left) turretTarget += 1;

            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addLine("======= Controls =======");
            telemetry.addLine(" Joysticks to move robot");
            telemetry.addLine("  dpad R/L to move turret");
            telemetry.update();
        }
    }
}