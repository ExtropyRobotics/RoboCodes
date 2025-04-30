package pedroPathing.auto;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Autonomous(name = "!TestSpline")
public class TestSplineAUTO extends OpMode{

    private Follower follower;
    private PathChain spline;
    private final Pose startPose = new Pose(0,0, Math.toRadians(0));
    private final Pose innerPoint1 = new Pose(48, 0, Math.toRadians(0));
    private final Pose innerPoint2 = new Pose(48, -48, Math.toRadians(0));
    private final Pose endPose = new Pose(0, -48, Math.toRadians(0));

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        spline = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(innerPoint1), new Point(innerPoint2), new Point(endPose)))
                .build();

        follower.followPath(spline);
    }

    @Override
    public void loop() {
        follower.update();
    }
}