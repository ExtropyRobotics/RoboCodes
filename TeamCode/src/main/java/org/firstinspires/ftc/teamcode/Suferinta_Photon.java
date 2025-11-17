package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.auto.RegioLeft;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Photon
@TeleOp(name = "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1Suferinta_Photon")
public class Suferinta_Photon extends LinearOpMode {
    SampleMecanumDrive drive;
    ArmControler_Photon brat;
    LynxModule control;

    int targetAx;
    int targetSlider;
    double parallelOffset = 0.497;
    double clawPos = 0.15;
    double multiplier = 1;


    int isRotating = 0;
    int isSliding = 0;

    boolean parallelToggle = false;
    boolean parallelOnce = false;

    boolean clawToggle = false;
    boolean clawOnce = false;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime time = new ElapsedTime();
        double lastTime = 0;

        PhotonLynxVoltageSensor sensor = hardwareMap.getAll(PhotonLynxVoltageSensor.class).iterator().next();
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        brat = new ArmControler_Photon(hardwareMap, telemetry);

        control = hardwareMap.get(LynxModule.class,"Control Hub");

        while(opModeInInit()){
            telemetry.addData("Voltage : ", control.getInputVoltage(VoltageUnit.MILLIVOLTS));
            telemetry.update();
            parallelOffset = 0.497;
        }

        waitForStart();

        lastTime = time.time();

        brat.setPower(0.3);
        brat.setPowerSlider(0.3);
        double voltage;

        while (opModeIsActive()) {
            voltage = sensor.getCachedVoltage();

            telemetry.addData("time0 ", 1/(time.time()-lastTime));
            lastTime = time.time();



           telemetry.addData("Battery Voltage ", voltage);
//
            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));
//
//            // set the wrist to be parallel to ground or parallel to high chamber or wall
//            if(gamepad2.right_bumper){
//                if(parallelToggle) {
//                    if(parallelOnce) {
//                        parallelOffset = 0.5672;
//                        multiplier = 1;
//                    }
//                    else{
//                        parallelOffset = 0.497;
//                        multiplier = 0;
//                    }
//                    parallelOnce = !parallelOnce;
//                    parallelToggle = false;
//                }
//            } else parallelToggle = true;
//
//            if(gamepad2.y){
//                clawPos = 0;
//                clawOnce = false;
//                clawToggle = false;
//                parallelOnce = false;
//                parallelOffset = 0.5178;
//                multiplier = 1;
//                targetAx = 100;
//            }
//
//            if(gamepad2.x){
//                clawPos = 0.15;
//                parallelOnce = true;
//                parallelOffset = 0.497;
//                multiplier = 0;
//                targetAx = 450;
//                clawOnce = true;
//                clawToggle = false;
//            }
//
//            brat.setWristParalel(parallelOffset, multiplier);
//
//            // toggle the claw from closed to open
//            if(gamepad2.a){
//                if(!clawToggle){
//                    if(clawOnce) clawPos = 0;
//                    else clawPos = 0.15;
//
//                    clawOnce = !clawOnce;
//                    clawToggle = true;
//                }
//            } else clawToggle = false;
//
//            brat.setClaw(clawPos);
//
//
            if(gamepad2.left_stick_y < 0 && targetAx < 900) targetAx += 20;
            if(gamepad2.left_stick_y > 0 && targetAx > 0) targetAx -= 20;
            if(gamepad2.right_stick_y < 0 && targetSlider < 2100) targetSlider += 40;
            if(gamepad2.right_stick_y > 0 && targetSlider > 0) targetSlider -= 40;

            if(gamepad2.dpad_up && targetAx < 900) targetAx += 5;
            if(gamepad2.dpad_down && targetAx > 0) targetAx -= 5;
            if(gamepad2.dpad_left && targetSlider < 2100) targetSlider += 5;
            if(gamepad2.dpad_right && targetSlider > 0) targetSlider -= 5;


            isRotating = brat.setAxPoz(targetAx);
            isSliding = brat.setSliderPoz(targetSlider);

//            brat.callTelemetry();
            telemetry.update();
        }
    }
}