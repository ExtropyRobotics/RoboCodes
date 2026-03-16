//package org.firstinspires.ftc.teamcode.TeleOPs.newRobot;
//
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//
//public class albaTeleOp extends LinearOpMode {
//
//    DcMotorEx turret;
//    Limelight3A limelight;
//    SampleMecanumDrive drive;
//    DcMotorEx plateEncoder;
//    CRServo plateServoLeft;
//    CRServo plateServoRight;
//    DcMotorEx intake;
//    DcMotorEx outtake;
//    DcMotorEx outtake2;
//    NormalizedColorSensor colorSensor;
//    DistanceSensor distanceSensor;
//    IMU imu;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        drive = new SampleMecanumDrive(hardwareMap);
//
//        plateServoLeft = hardwareMap.get(CRServo.class, "plateServoLeft");
//        plateServoRight = hardwareMap.get(CRServo.class, "plateServoRight");
//
//        intake = hardwareMap.get(DcMotorEx.class, "intake");
//        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
//        turret = hardwareMap.get(DcMotorEx.class, "turret");
//
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
//        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
//
//        imu = hardwareMap.get(IMU.class, "imu");
//
//        plateEncoder = hardwareMap.get(DcMotorEx.class, "intake");
//
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//
//        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
//
//        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
//        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        waitForStart();
//
//        while ()
//
//    }
//}
