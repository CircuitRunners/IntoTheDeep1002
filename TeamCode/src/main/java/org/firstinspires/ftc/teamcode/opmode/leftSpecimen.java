package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.config.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.config.subsystems.Arm;
import org.firstinspires.ftc.teamcode.config.subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.config.util.action.Action;
import org.firstinspires.ftc.teamcode.config.util.action.Actions;
import org.firstinspires.ftc.teamcode.config.util.action.ParallelAction;
import org.firstinspires.ftc.teamcode.config.util.action.SequentialAction;
import org.firstinspires.ftc.teamcode.config.util.action.SleepAction;


@Autonomous
public class leftSpecimen extends OpMode{
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private Arm arm;

    private EndEffector endEffector;

    // Define key poses
    private Pose startPosition = new Pose(7.5, 64, Math.toRadians(180));
    private Pose specimen = new Pose(35, 64, Math.toRadians(180));
    private Pose specimenScorePos = new Pose(37, 64, Math.toRadians(180));
    private Pose specimenCycleLineUp = new Pose(30, 64, Math.toRadians(180));
    private Pose parkPos = new Pose(10, 64, Math.toRadians(180));
    private Pose cycleSpecimen1Pos = new Pose(32, 26, Math.toRadians(180));
    private Pose cycleSpecimenObs1Pos = new Pose(17, 26, Math.toRadians(0));
    private Pose cycleSpecimen2Pos = new Pose(30, 16, Math.toRadians(180));
    private Pose cycleSpecimenObs2Pos = new Pose(17, 15, Math.toRadians(0));
    private Pose grabSpecimen1 = new Pose(17, 24, Math.toRadians(0));
    private Pose specimenPos2 = new Pose(35, 65, Math.toRadians(180));
    private Pose specimenScorePos2 = new Pose(37, 65, Math.toRadians(180));
    private Pose specimenPos3 = new Pose(35, 66, Math.toRadians(180));
    private Pose specimenScorePos3 = new Pose(37, 66, Math.toRadians(180));
//    private Pose cycleSpecimen2Pos = new Pose(32, y, Math.toRadians(180));
//    private Pose cycleSpecimenObs2Pos = new Pose(17, y, Math.toRadians(0));55566    private Pose parkPosFinal = new Pose(10, 40, Math.toRadians(180));



    private PathChain specimenPath, parkPath, scorePath, finalParkPath, cycleSpecimen1, cycleSpecimenObs1, cycleSpecimen2, cycleSpecimenObs2, specimenCycleLineUpPath, grabSpecimen1Path, specimenPos2Path, specimenScorePos2Path, grabSpecimen2Path, specimenPos3Path, specimenScorePos3Path;
    public void buildPaths() {
        specimenPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPosition), new Point(specimen)))
                .setConstantHeadingInterpolation(specimen.getHeading())
                .build();
        scorePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen), new Point(specimenScorePos)))
                .setConstantHeadingInterpolation(specimenScorePos.getHeading())
                .build();
//        parkPath = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(specimen), new Point(parkPos)))
//                .setConstantHeadingInterpolation(parkPos.getHeading())
//                .build();
//        finalParkPath = follower.pathBuilder()
//
//                .setConstantHeadingInterpolation(parkPosFinal.getHeading())
//                .build();
        cycleSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenScorePos), new Point(cycleSpecimen1Pos)))
                .setConstantHeadingInterpolation(cycleSpecimen1Pos.getHeading())
                .build();
        cycleSpecimenObs1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(cycleSpecimen1Pos), new Point(cycleSpecimenObs1Pos)))
                .setConstantHeadingInterpolation(cycleSpecimenObs1Pos.getHeading())
                .build();
        cycleSpecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(cycleSpecimenObs1Pos), new Point(cycleSpecimen2Pos)))
                .setConstantHeadingInterpolation(cycleSpecimen2Pos.getHeading())
                .build();
        cycleSpecimenObs2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(cycleSpecimen2Pos), new Point(cycleSpecimenObs2Pos)))
                .setConstantHeadingInterpolation(cycleSpecimenObs2Pos.getHeading())
                .build();
        specimenCycleLineUpPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenScorePos), new Point(specimenCycleLineUp)))
                .setConstantHeadingInterpolation(specimenCycleLineUp.getHeading())
                .build();
        grabSpecimen1Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(cycleSpecimenObs2Pos), new Point(grabSpecimen1)))
                .setConstantHeadingInterpolation(grabSpecimen1.getHeading())
                .build();
        specimenPos2Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabSpecimen1), new Point(specimenPos2)))
                .setConstantHeadingInterpolation(specimenPos2.getHeading())
                .build();
        specimenScorePos2Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPos2), new Point(specimenScorePos2)))
                .setConstantHeadingInterpolation(specimenScorePos2.getHeading())
                .build();
        grabSpecimen2Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenScorePos2), new Point(grabSpecimen1)))
                .setConstantHeadingInterpolation(grabSpecimen1.getHeading())
                .build();
        specimenPos3Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabSpecimen1), new Point(specimenPos3)))
                .setConstantHeadingInterpolation(specimenPos3.getHeading())
                .build();
        specimenScorePos3Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPos3), new Point(specimenScorePos3)))
                .setConstantHeadingInterpolation(specimenScorePos3.getHeading())
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
                    Actions.runBlocking(preSpecimenScore());
//                    Actions.runBlocking(arm.autoArmPreSpecimen);
                    follower.followPath(specimenPath);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    Actions.runBlocking(new SleepAction(0.5));
                    Actions.runBlocking(specimenScore());
//                    Actions.runBlocking(endEffector.openClaw);
                    follower.followPath(scorePath);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(specimenCycleLineUpPath);
                    Actions.runBlocking(endEffector.openClaw);
                    Actions.runBlocking(new SleepAction(1));
                    //follower.followPath(parkPath);
                    setPathState(5);
                }
                break;
//            case 4:
//                if (!follower.isBusy()) {
//                    Actions.runBlocking(new SleepAction(1));
//                    follower.followPath(finalParkPath);
//                    setPathState(5);
//                }
            case 5:
                if (!follower.isBusy()) {
                    Actions.runBlocking(arm.armMax);
                    Actions.runBlocking(endEffector.diffyIdle);
                    follower.followPath(cycleSpecimen1);

                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    Actions.runBlocking(endEffector.diffyObs);
                    Actions.runBlocking(new SleepAction(0.5));
                    Actions.runBlocking(endEffector.closeClaw);
                    Actions.runBlocking(new SleepAction(0.5));
                    Actions.runBlocking(endEffector.diffyClear);
                    Actions.runBlocking(endEffector.closeClaw);
                    follower.followPath(cycleSpecimenObs1);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    Actions.runBlocking(endEffector.openClaw);
                    Actions.runBlocking(endEffector.diffyIdle);
                    follower.followPath(cycleSpecimen2);
                    Actions.runBlocking(endEffector.openClaw);
                    Actions.runBlocking(endEffector.diffyObs);
                    Actions.runBlocking(new SleepAction(0.5));
                    Actions.runBlocking(endEffector.closeClaw);
                    Actions.runBlocking(endEffector.diffyIdle);
                    setPathState(9);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(cycleSpecimenObs2);
                    Actions.runBlocking(endEffector.openClaw);
                    Actions.runBlocking(endEffector.diffyIdle);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(grabSpecimen1Path);
                    Actions.runBlocking(endEffector.diffyObs);
                    Actions.runBlocking(endEffector.closeClaw);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    Actions.runBlocking(preSpecimenScore());
//                    Actions.runBlocking(arm.autoArmPreSpecimen);
                    follower.followPath(specimenPos2Path);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    Actions.runBlocking(new SleepAction(0.5));
                    Actions.runBlocking(specimenScore());
//                    Actions.runBlocking(endEffector.openClaw);
                    follower.followPath(specimenScorePos2Path);
                    setPathState(12);
                }
                break;
            case 12:
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
        telemetry.addData("Arm Target", arm.getArmTarget());
        telemetry.addData("Arm Pos", arm.armAngle());
        telemetry.addData("Diffy1 Position", "%.2f", endEffector.getDiffy1Position());
        telemetry.addData("Diffy2 Position", "%.2f", endEffector.getDiffy2Position());
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

    public Action preSpecimenScore() {
        return new SequentialAction(
                //new SleepAction(0.5),
                endEffector.closeClaw,
                new ParallelAction(
                        endEffector.closeClaw,
                      //  endEffector.autoPreSpecimen,
                        arm.autoArmPreSpecimen
                ),
                endEffector.closeClaw
                //new SleepAction(0.5)
        );
    }

    public Action specimenScore() {
        return new SequentialAction(
                endEffector.closeClaw,
                // new SleepAction(0.5),
                endEffector.autoSpecimen,
                arm.autoArmSpecimen,
                new SleepAction(1)
                // new SleepAction(3)
        );
    }

}
