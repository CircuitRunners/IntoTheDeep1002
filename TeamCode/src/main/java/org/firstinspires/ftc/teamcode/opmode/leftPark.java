package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.config.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.Timer;

@Autonomous
public class leftPark extends OpMode{
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Define key poses
    private Pose startPosition = new Pose(7.5, 80, Math.toRadians(180));
    private Pose parkPos = new Pose(7.5, 40, Math.toRadians(180));

    private PathChain cycle;
    public void buildPaths() {
        cycle = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPosition), new Point(parkPos)))
                .setConstantHeadingInterpolation(parkPos.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(cycle);
                setPathState(1);
                break;
            case 1:
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
        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }


}
