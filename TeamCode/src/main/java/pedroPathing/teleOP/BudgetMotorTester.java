package pedroPathing.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "pew")
public class BudgetMotorTester extends LinearOpMode{
    DcMotor chestie = null;
    DcMotor chestie2 = null;

    @Override
    public void runOpMode() throws InterruptedException {

        chestie = hardwareMap.get(DcMotor.class, "pew");
        chestie2 = hardwareMap.get(DcMotor.class, "pewpew");

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.y){
                chestie.setPower(1);
                chestie2.setPower(-1);
            }
            if(gamepad1.a){
                chestie.setPower(-1);
                chestie2.setPower(1);
            }
            if(gamepad1.x){
                chestie.setPower(0);
                chestie2.setPower(0);
            }
            if(gamepad1.dpad_up){
                chestie.setPower(1);
                chestie2.setPower(1);
            }
            if(gamepad1.dpad_down){
                chestie.setPower(-1);
                chestie2.setPower(-1);
            }
        }
    }
}