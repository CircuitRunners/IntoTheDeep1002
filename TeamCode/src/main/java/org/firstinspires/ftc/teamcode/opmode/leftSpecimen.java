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
import org.firstinspires.ftc.teamcode.config.util.action.Action;
import org.firstinspires.ftc.teamcode.config.util.action.Actions;
import org.firstinspires.ftc.teamcode.config.util.action.ParallelAction;
import org.firstinspires.ftc.teamcode.config.util.action.SequentialAction;
import org.firstinspires.ftc.teamcode.config.util.action.SleepAction;


@Autonomous
public class leftBucket extends OpMode{
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private Arm arm;

    private EndEffector endEffector;

    // Define key poses
    private Pose startPosition = new Pose(7.5, 80, Math.toRadians(180));
    private Pose bucketClear = new Pose(20, 110, Math.toRadians(315));
    private Pose bucketPos = new Pose(6.2, 125,Math.toRadians(315));
    private Pose preprePark = new Pose(7,105,Math.toRadians(0));
    private Pose prePark = new Pose(48,105,Math.toRadians(270));
    private Pose parkPos = new Pose(55, 102, Math.toRadians(270));


    private PathChain basketClear, score, park;
    public void buildPaths() {
        basketClear = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPosition), new Point(bucketClear)))
                .setConstantHeadingInterpolation(bucketClear.getHeading())
                .build();
        score = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucketClear), new Point(bucketPos)))
                .setConstantHeadingInterpolation(bucketPos.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucketPos), new Point(preprePark)))
                .setConstantHeadingInterpolation(preprePark.getHeading())
                .addPath(new BezierLine(new Point(preprePark), new Point(prePark)))
                .setConstantHeadingInterpolation(prePark.getHeading())
                .addPath(new BezierLine(new Point(prePark), new Point(parkPos)))
                .setConstantHeadingInterpolation(parkPos.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                Actions.runBlocking(endEffector.closeClaw);
                Actions.runBlocking(endEffector.diffyIdle);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    Actions.runBlocking(endEffector.closeClaw);
                    follower.followPath(basketClear);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    Actions.runBlocking(endEffector.closeClaw);
                    Actions.runBlocking(arm.armLowBasket);
                    Actions.runBlocking(endEffector.diffyBasket);
//                    Actions.runBlocking(new SleepAction(1));
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    Actions.runBlocking(endEffector.closeClaw);
                    follower.followPath(score);
                    setPathState(4);
                }
                break;
            case 4:
              if (!follower.isBusy()) {
                  Actions.runBlocking(endEffector.openClaw);
                  setPathState(5);
              }
              break;
            case 5:
                if (!follower.isBusy()) {
                    Actions.runBlocking(resetArm());
                    follower.followPath(park);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    Actions.runBlocking(arm.armSpecimenScore);
                    Actions.runBlocking(new SleepAction(5));
                    setPathState(7);
                }
            case 7:
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
        endEffector.closeClaw();
        endEffector.idlePosition();
        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

    public Action resetArm() {
        return new SequentialAction(
                new SleepAction(1),
                new ParallelAction(
                        endEffector.diffyIdle,
                        arm.armObservation
                ),
                arm.armUpright
        );
    }

}
