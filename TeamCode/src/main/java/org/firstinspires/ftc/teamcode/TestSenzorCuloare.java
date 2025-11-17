package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Test Color Sensor", group = "Test")
public class TestSenzorCuloare extends LinearOpMode {

    ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Inițializare senzor
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        waitForStart();

        while (opModeIsActive()) {
            // Citește valorile brute
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();
            int alpha = colorSensor.alpha(); // intensitatea luminii totale

            telemetry.addData("Roșu", red);
            telemetry.addData("Verde", green);
            telemetry.addData("Albastru", blue);
            telemetry.addData("Luminozitate", alpha);

            // Determină culoarea dominantă
            if (green > red && green > blue) {
                telemetry.addLine("Culoare detectată: VERDE");
            } else if (red > green && blue > green) {
                telemetry.addLine("Culoare detectată: MOV");
            } else {
                telemetry.addLine("alta culoare");
            }

            telemetry.update();
        }
    }
}

