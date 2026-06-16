package org.firstinspires.ftc.teamcode.TeleOPs.Tester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;


@Disabled
@TeleOp(name = "imuCorrectionTurret", group = "Tester")
public class imuCorrectionTurret extends LinearOpMode {

    IMU imu = null;
    Limelight3A limelight = null;

    YawPitchRollAngles angles;
    Servo launch;
    double x,y,z;

    double posServo = 0.5;

    static double g = 9.81;
    static double m = 0.075;
    static double diameter = 0.127;
    static double r = diameter / 2.0;

    static double rho = 1.225;
    static double Cd = 0.48;
    static double area = Math.PI * diameter * diameter / 4.0;
    static double D = 0.5 * rho * Cd * area;

    static double turretHeight = 0.3;

    // Target
    static double targetX = 2.0;
    static double targetY = 1.1;

    static double v0 = 20; // fixed speed (change as needed)

    public void runOpMode() throws InterruptedException{

        // Constants



        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class,"imu");
        launch = hardwareMap.get(Servo.class, "launch");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));



        limelight.pipelineSwitch(0);
        limelight.start();


        waitForStart();


        while(opModeIsActive()){
            //double angle = 90 - posServo * 65 / 0.93;

            telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);

            angles = imu.getRobotYawPitchRollAngles();

            x = angles.getYaw();
            y = angles.getPitch();
            z = angles.getRoll();

            limelight.updateRobotOrientation(angles.getYaw(AngleUnit.DEGREES));

            launch.setPosition(posServo);

            if(gamepad1.left_stick_y > 0) posServo += 0.01;
            if(gamepad1.left_stick_y < 0) posServo -= 0.01;

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botPose = result.getBotpose_MT2();


                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    int id = tags.get(0).getFiducialId();
                    double skew = tags.get(0).getSkew();

                    Pose3D roboPosField = tags.get(0).getRobotPoseFieldSpace();
                    Pose3D roboPosTarget = tags.get(0).getRobotPoseFieldSpace();

                    telemetry.addData("distance", getDistance(tags.get(0).getTargetArea()));

                    targetX = getDistance(getDistance(tags.get(0).getTargetArea())) / 1000;
                    Result r = solve();

                    telemetry.addData("optimal angle", r.angle);
                    telemetry.addData("optimal speed" , r.speed);

                    telemetry.addData("botPos ", botPose.toString());

                    telemetry.addData("target x ", result.getTx());
                    telemetry.addData("target area ", result.getTa());

                    telemetry.addData("robo pos field x", roboPosField.getPosition().x);
                    telemetry.addData("robo pos field y",roboPosField.getPosition().y);
                    telemetry.addData("robo pos target x",roboPosTarget.getPosition().x);
                    telemetry.addData("robo pos target y",roboPosTarget.getPosition().y);


                    telemetry.addData("id", id);
                    telemetry.addData("skew", skew);

                }
            }


//            telemetry.addData("servo angle" , angle);
            telemetry.addData("servo pos" , posServo);
            telemetry.addData("imu yaw", x);
            telemetry.addData("imu pitch", y);
            telemetry.addData("imu roll", z);
            telemetry.update();

        }


    }
    static Result solve() {

        double lowSpeed = 5;
        double highSpeed = 20;

        for (int i = 0; i < 40; i++) {

            double midSpeed = (lowSpeed + highSpeed) / 2.0;

            double angle = findAngle(midSpeed);

            if (Double.isNaN(angle))
                lowSpeed = midSpeed; // can't reach → need more speed
            else
                highSpeed = midSpeed;
        }

        double finalSpeed = (lowSpeed + highSpeed) / 2.0;
        double finalAngle = findAngle(finalSpeed);

        return new Result(finalAngle, finalSpeed);
    }
    static double findAngle(double v0) {

        double low = 40;
        double high = 65;

        double fLow = simulate(low, v0);
        double fHigh = simulate(high, v0);

        if (Double.isNaN(fLow) || Double.isNaN(fHigh))
            return Double.NaN;

        for (int i = 0; i < 40; i++) {

            double mid = (low + high) / 2.0;
            double x = simulate(mid, v0);

            if (Double.isNaN(x))
                return Double.NaN;

            if (x < targetX)
                low = mid;
            else
                high = mid;
        }

        return (low + high) / 2.0;
    }
    static double simulate(double angleDeg, double v0) {

        double theta = Math.toRadians(angleDeg);

        double x = 0;
        double y = turretHeight;
        double vx = v0 * Math.cos(theta);
        double vy = v0 * Math.sin(theta);

        double dt = 0.01;

        while (true) {

            double v = Math.sqrt(vx * vx + vy * vy);

            double ax = -(D / m) * v * vx;
            double ay = -g - (D / m) * v * vy;

            vx += ax * dt;
            vy += ay * dt;
            x += vx * dt;
            y += vy * dt;

            // crossed target height downward
            if (y <= targetY && vy < 0)
                return x;

            // fell to ground before reaching height
            if (y < 0)
                return Double.NaN;
        }
    }
    static class Result {
        double angle, speed;
        Result(double a, double s){ angle=a; speed=s; }
    }

    double getDistance(double ta){
        double scale = 24446.5;

        return Math.pow(scale/ta,0.511);   // lower here is lower distance
    }
}