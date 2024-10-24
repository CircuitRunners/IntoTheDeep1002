package org.firstinspires.ftc.teamcode.opmode.Archive;

import org.firstinspires.ftc.teamcode.config.subsystems.Arm;
import org.firstinspires.ftc.teamcode.config.subsystems.EndEffector;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.config.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.Timer;

import org.firstinspires.ftc.teamcode.config.util.action.Actions;

@Disabled
@Autonomous
public class LinearAuto extends OpMode{
    private Follower follower;

    private Arm arm;
    private EndEffector endEffector;
    private Timer pathTimer;
    private int pathState;



    // Define key poses
    private Pose startPosition = new Pose(0, 0, 0);
    private Pose spike = new Pose(10, 0, 0);
//    private Pose stack = new Pose(60, 20, 0);
//    private Pose park = new Pose(60, 100, 0);
//    private Pose score = new Pose(35, 100, 0);

    private PathChain cycle;
    public void buildPaths() {
        cycle = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPosition), new Point(spike)))
                .setConstantHeadingInterpolation(spike.getHeading())
//                .addPath(new BezierLine(new Point(spike), new Point(stack)))
//                .setConstantHeadingInterpolation(stack.getHeading())
//                .addPath(new BezierLine(new Point(stack), new Point(park)))
//                .setConstantHeadingInterpolation(park.getHeading())
//                .addPath(new BezierLine(new Point(park), new Point(score)))
//                .setConstantHeadingInterpolation(score.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                Actions.runBlocking(arm.armSpecimen);
                Actions.runBlocking(endEffector.diffySpecimen);
                follower.followPath(cycle);
                setPathState(1);
                break;
            case 2:
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
        arm.update();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
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
