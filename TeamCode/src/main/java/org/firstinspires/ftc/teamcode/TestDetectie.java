package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "AprilTag si Culoare")
public class TestDetectie extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {
        //  Creează procesorul pentru AprilTag
        aprilTag = new AprilTagProcessor.Builder().build();



        //  Creează Vision Portal (sistemul de cameră)
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Camera pornită. Apasă START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            detectAprilTags();
            telemetry.update();
        }
    }

    private void detectAprilTags() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.size() > 0) {
            AprilTagDetection tag = detections.get(0);
            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("Pozitie X", tag.ftcPose.x);
            telemetry.addData("Pozitie Y", tag.ftcPose.y);
        } else {
            telemetry.addLine("Niciun AprilTag detectat");
        }
    }
}

