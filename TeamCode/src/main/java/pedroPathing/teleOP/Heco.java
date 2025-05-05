package pedroPathing.teleOP;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "HecobotTeleOP")
public class Heco extends LinearOpMode {

    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    DcMotor launcher1 = null;
    DcMotor launcher2 = null;
    DcMotor slider = null;
    int targetSlider = 0;
    boolean launcherToggle = false;
    boolean launched = false;

    @Override
    public void runOpMode() {

        launcher1 = hardwareMap.get(DcMotor.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotor.class, "launcher2");
        slider = hardwareMap .get(DcMotor.class, "slider");

        initMotor(slider, DcMotorSimple.Direction.REVERSE);

        launcher1.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);

        slider.setTargetPosition(targetSlider);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        waitForStart();
        slider.setPower(1);

        follower.startTeleopDrive();

        while(opModeIsActive()){
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();

            if(gamepad2.right_stick_y > 0) targetSlider -= 5;
            if(gamepad2.right_stick_y < 0) targetSlider += 5;

            if(targetSlider < 0) targetSlider = 0;
            if(targetSlider > 1100) targetSlider = 1100;

            if(gamepad2.a){
                if(!launcherToggle){
                    if(!launched) {
                        launcher1.setPower(-1);
                        launcher2.setPower(1);
                    } else{
                        launcher1.setPower(0);
                        launcher2.setPower(0);
                    }

                    launched = !launched;
                    launcherToggle = true;
                }
            } else launcherToggle = false;

            slider.setTargetPosition(targetSlider);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("slider", targetSlider);
            telemetry.update();
        }
    }
    private void initMotor(DcMotor motor, DcMotorSimple.Direction direction){
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}