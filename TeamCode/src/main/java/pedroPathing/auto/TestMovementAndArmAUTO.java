package pedroPathing.auto;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Autonomous(name = "!TestMovementAndArm")
public class TestMovementAndArmAUTO extends OpMode{

   DcMotor axUp = null;
   DcMotor axDown =null;
   DcMotor slider = null;

   int targetPozAx = 0;
   int targePozSlider = 0;

    private Follower follower;
    private PathChain backAndForth;
    private final Pose startPose = new Pose(0,0, Math.toRadians(0));
    private final Pose endPose = new Pose(24, 0, Math.toRadians(0));

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        axUp = hardwareMap.get(DcMotor.class, "axUp");
        axDown = hardwareMap.get(DcMotor.class, "axDown");
        slider = hardwareMap.get(DcMotor.class, "brat");

        axUp.setDirection(DcMotorSimple.Direction.REVERSE);
        axDown.setDirection(DcMotorSimple.Direction.FORWARD);
        slider.setDirection(DcMotorSimple.Direction.REVERSE);

        axUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        axDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        axUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        axDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        axUp.setTargetPosition(targetPozAx);
        axDown.setTargetPosition(targetPozAx);
        slider.setTargetPosition(targePozSlider);

        axUp.setPower(0.4);
        axDown.setPower(0.4);
        slider.setPower(0.4);

        backAndForth = follower.pathBuilder()
                .addTemporalCallback(0, ()->{
                    heil(600, 500);
                })
                .addTemporalCallback(0.5, ()->{
                    heil(0, 0);
                })
                .addPath(new BezierLine(new Point(startPose), new Point(endPose)))
                .addPath(new BezierLine(new Point(endPose), new Point(startPose)))
                .build();

        follower.followPath(backAndForth);
    }

    public void heil(int axPoz, int sliderPoz){
        targetPozAx = axPoz;
        targePozSlider = sliderPoz;
        axUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        follower.update();
    }
}