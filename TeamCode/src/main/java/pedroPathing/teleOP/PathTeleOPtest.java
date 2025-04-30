package pedroPathing.teleOP;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "PathTeleOPtest")
public class PathTeleOPtest extends LinearOpMode {
    private Follower follower;
    private PathChain circle;
    private double RADIUS = 10;
    private boolean circleToggle = false;
    private final Pose startPose = new Pose(0,0,0);

    @Override
    public void runOpMode() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        circle = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(0,0, Point.CARTESIAN), new Point(RADIUS,0, Point.CARTESIAN), new Point(RADIUS, RADIUS, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(RADIUS, RADIUS, Point.CARTESIAN), new Point(RADIUS,2*RADIUS, Point.CARTESIAN), new Point(0,2*RADIUS, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(0,2*RADIUS, Point.CARTESIAN), new Point(-RADIUS,2*RADIUS, Point.CARTESIAN), new Point(-RADIUS, RADIUS, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(-RADIUS, RADIUS, Point.CARTESIAN), new Point(-RADIUS,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)))
                .build();

        waitForStart();
        follower.startTeleopDrive();
        while(opModeIsActive()){
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();

            if(gamepad1.b){
                if(!circleToggle){
                    follower.followPath(circle);
                    circleToggle = true;
                }
            } else circleToggle = false;

            follower.startTeleopDrive();
        }
    }
}