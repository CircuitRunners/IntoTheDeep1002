package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.config.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.config.subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.config.subsystems.Arm;
import org.firstinspires.ftc.teamcode.config.util.action.Actions;

@Autonomous
public class blueLeftBucket extends OpMode{
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private Arm arm;

    private EndEffector endEffector;

    // Define key poses
    private Pose startPosition = new Pose(7.5, 80, Math.toRadians(180));
    private Pose bucketClear = new Pose(11, 121, Math.toRadians(315));
    private Pose bucketPos = new Pose(6.2, 125,Math.toRadians(315));
    private Pose parkPos = new Pose(7.5, 40, Math.toRadians(180));


    private PathChain basketClear, score;
    public void buildPaths() {
        basketClear = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPosition), new Point(bucketClear)))
                .setConstantHeadingInterpolation(bucketClear.getHeading())
                .build();
        score = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucketClear), new Point(bucketPos)))
                .setConstantHeadingInterpolation(bucketPos.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(basketClear);
                setPathState(1);
                break;
            case 1:
                Actions.runBlocking(arm.armLowBasket);
                follower.followPath(score);
                setPathState(2);
            case 2:
                follower.followPath(score);
                setPathState(3);
            case 3:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
            default:
                requestOpModeStop();
                break;

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        arm.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toRadians(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPosition);
        arm = new Arm(hardwareMap);
        endEffector = new EndEffector(hardwareMap);
        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }


}
