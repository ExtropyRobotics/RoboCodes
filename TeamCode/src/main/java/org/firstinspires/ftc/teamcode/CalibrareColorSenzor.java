package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "CalibrareColorSenzor")
public class CalibrareColorSenzor extends LinearOpMode {

    ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        waitForStart();

        while (opModeIsActive()) {
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();
            int alpha = colorSensor.alpha();

            telemetry.addData("Roșu", red);
            telemetry.addData("Verde", green);
            telemetry.addData("Albastru", blue);
            telemetry.addData("Luminozitate", alpha);
            telemetry.update();
        }
    }
}
