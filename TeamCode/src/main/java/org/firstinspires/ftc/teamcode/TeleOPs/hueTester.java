package org.firstinspires.ftc.teamcode.TeleOPs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp (name = "hueTester")
public class hueTester extends LinearOpMode {
    NormalizedColorSensor colorSensor = null;
    NormalizedRGBA colors;
    @Override
    public void runOpMode() throws InterruptedException {

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");

        waitForStart();

        while(opModeIsActive()){

            colors = colorSensor.getNormalizedColors();

            double hue = 0;

            double max = Math.max(colors.red, colors.blue);
            double min = Math.min(colors.red, colors.blue);

            max = Math.max(max, colors.green);
            min = Math.min(min, colors.green);

            double delta = max - min;

            if(delta == 0) hue = 0;

            if(max == colors.red) hue = 60 * (((colors.green - colors.blue) / delta) % 6);
            if(max == colors.green) hue = 60 * ((colors.blue - colors.red) / delta);
            if(max == colors.blue) hue = 60 * ((colors.red - colors.green) / delta);

            if(hue < 0) hue += 360;

            telemetry.addData("Hue:", hue);
            telemetry.update();
        }
    }
}