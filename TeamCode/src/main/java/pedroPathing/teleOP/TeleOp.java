package pedroPathing.teleOP;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "!TeleOP")
public class TeleOp extends LinearOpMode{
    ArmControler brat;
    LynxModule control;
    Follower follower;
    int targetAx;
    int targetSlider;
    double parallelOffset = 0;
    double clawPos = 0.15;
    double multiplier = 1;
    int isRotating = 0;
    int isSliding = 0;
    boolean parallelToggle = false;
    boolean parallelOnce = false;
    boolean clawToggle = false;
    boolean clawOnce = false;
    private final Pose startPose = new Pose(0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        brat = new ArmControler(hardwareMap);

        control = hardwareMap.get(LynxModule.class,"Control Hub");

        while(opModeInInit()){
            parallelOffset = 0;
        }

        waitForStart();

        follower.startTeleopDrive();
        brat.setPower(1);
        brat.setPowerSlider(1);

        while (opModeIsActive()) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();

            if(gamepad2.right_bumper){
                if(parallelToggle) {
                    if(parallelOnce) {
                        parallelOffset = 0.4;
                        multiplier = 1;
                    }
                    else{
                        parallelOffset = 0;
                        multiplier = 0;
                    }
                    parallelOnce = !parallelOnce;
                    parallelToggle = false;
                }
            } else parallelToggle = true;


            if(gamepad2.y){
                clawPos = 0;
                clawOnce = false;
                clawToggle = false;
                parallelOnce = false;
                parallelOffset = 0.1485;
                multiplier = 0;
                targetAx = 120;
            }

            if(gamepad2.x){
                clawPos = 0.15;
                parallelOnce = true;
                parallelOffset = 0;
                multiplier = 0;
                targetAx = 460;
                clawOnce = true;
                clawToggle = false;
            }

            brat.setWristParalel(parallelOffset, multiplier);

            if(gamepad2.a){
                if(!clawToggle){
                    if(clawOnce) clawPos = 0;
                    else clawPos = 0.15;

                    clawOnce = !clawOnce;
                    clawToggle = true;
                }
            } else clawToggle = false;

            brat.setClaw(clawPos);

            if(gamepad2.left_stick_y < 0 && targetAx < 900) targetAx += 20;
            if(gamepad2.left_stick_y > 0 && targetAx > 0) targetAx -= 20;
            if(gamepad2.right_stick_y < 0 && targetSlider < 2100) targetSlider += 40;
            if(gamepad2.right_stick_y > 0 && targetSlider > 0) targetSlider -= 40;

            if(gamepad2.dpad_up && targetAx < 900) targetAx += 5;
            if(gamepad2.dpad_down && targetAx > 0) targetAx -= 5;

            isRotating = brat.setAxPoz(targetAx);
            isSliding = brat.setSliderPoz(targetSlider);
        }
    }
}