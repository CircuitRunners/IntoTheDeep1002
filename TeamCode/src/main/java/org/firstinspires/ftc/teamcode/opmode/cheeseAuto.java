package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.config.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.config.subsystems.Arm;
import org.firstinspires.ftc.teamcode.config.subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.config.util.action.Action;
import org.firstinspires.ftc.teamcode.config.util.action.Actions;
import org.firstinspires.ftc.teamcode.config.util.action.ParallelAction;
import org.firstinspires.ftc.teamcode.config.util.action.SequentialAction;
import org.firstinspires.ftc.teamcode.config.util.action.SleepAction;


@Autonomous
public class cheeseAuto extends OpMode{
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private Arm arm;

    private EndEffector endEffector;

    // Define key poses
    private Pose startPosition = new Pose(7.5, 64, Math.toRadians(180));
    // private Pose specimen = new Pose(35, 64, Math.toRadians(180));
    private Pose specimenScorePos = new Pose(42.5, 68, Math.toRadians(180));

    private Pose specimen1ScorePos = new Pose(42.5, 74, Math.toRadians(180));
    private Pose bezPoint2 = new Pose(14, 74, Math.toRadians(90));
    private Pose bezPoint = new Pose(42.5, 24, Math.toRadians(90));
    private Pose specimen2ScorePos = new Pose(45, 78, Math.toRadians(180));

    private Pose specimenCycleLineUp = new Pose(20, 68, Math.toRadians(180));
    private Pose parkPos = new Pose(10, 64, Math.toRadians(180));
    private Pose cycleSpecimen1LineupPos = new Pose(22, 39.8, Math.toRadians(180));
    private Pose cycleSpecimen1IntakePos = new Pose(59, 38.8, Math.toRadians(180));
    private Pose cycleSPecimenBehindPos = new Pose(59,28.5, Math.toRadians(180));
    private Pose cycleSpecimen2BehindPos = new Pose(59,18, Math.toRadians(180));
    private Pose cycleSpecimenObs1Pos = new Pose(14, 30, Math.toRadians(180));
    private Pose backUpFromObsPos= new Pose(25, 18, Math.toRadians(180));
    private Pose turnObs1Pos = new Pose(20, 18, Math.toRadians(0));
    private Pose specimen3BehindPos = new Pose(59, 11, Math.toRadians(180));
    private Pose cycleSpecimen3Pos = new Pose(60, 11, Math.toRadians(180));
    private Pose cycleSpecimenObs3Pos = new Pose(15, 11, Math.toRadians(180));
    private Pose cycleSpecimenObs2Pos = new Pose(15, 18, Math.toRadians(180));
    private Pose lineUpGrabSpecimen = new Pose(20, 24, Math.toRadians(0));
    private Pose grabSpecimenPos = new Pose(8.75, 18, Math.toRadians(0)); //11.75 x og
    private Pose grabSpecimen1 = new Pose(17, 24, Math.toRadians(180));
    private Pose specimenScorePos2 = new Pose(37, 65, Math.toRadians(180));
    private Pose specimenPos3 = new Pose(35, 66, Math.toRadians(180));
    private Pose specimenScorePos3 = new Pose(37, 66, Math.toRadians(180));
//    private Pose cycleSpecimen2Pos = new Pose(32, y, Math.toRadians(180));
//    private Pose cycleSpecimenObs2Pos = new Pose(17, y, Math.toRadians(0));55566    private Pose parkPosFinal = new Pose(10, 40, Math.toRadians(180));
//pushing


    private PathChain specimenPath, scorePath, cycleSpecimen3Behind, parkPath,lineUpGrabSpecimen1, turnObs1,backUpFromObs, lineUpScoreSpecimen1, scoreSpecimen1, specimen1Behind,finalParkPath, cycleSpecimen1Lineup,cycleSpecimen1, cycleSpecimenObs1, cycleSpecimen2,cycleSpecimen2Behind, cycleSpecimenObs2, specimenCycleLineUpPath, grabSpecimen, lineUpGrabSpecimen3, specimenPos2Path, specimenScorePos2Path, grabSpecimen2Path, specimenPos3Path, specimenScorePos3Path, lineUpGrabSpecimenPath, specimenCycleLineUpPath2, cycleSpecimen3Path, cycleSpecimenObs3Path, cycledSpecimen1Path, cheeseAuto, cheeseCycleAuto, mergedPath;
    public void buildPaths() {
//        specimenPath = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(startPosition), new Point(specimen)))
//                .setConstantHeadingInterpolation(specimen.getHeading())
//                .build();
        scorePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPosition), new Point(specimenScorePos)))
                .setConstantHeadingInterpolation(specimenScorePos.getHeading())
                .build();
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenCycleLineUp), new Point(parkPos)))
                .setConstantHeadingInterpolation(parkPos.getHeading())
                .build();
//        finalParkPath = follower.pathBuilder()
//
//                .setConstantHeadingInterpolation(parkPosFinal.getHeading())
//                .build();
        cycleSpecimen1Lineup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenCycleLineUp), new Point(cycleSpecimen1LineupPos)))
                .setConstantHeadingInterpolation(cycleSpecimen1LineupPos.getHeading())
                .build();
        cycleSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(cycleSpecimen1LineupPos), new Point(cycleSpecimen1IntakePos)))
                .setConstantHeadingInterpolation(cycleSpecimen1IntakePos.getHeading())
                .build();
        specimen1Behind = follower.pathBuilder()
                .addPath(new BezierLine(new Point(cycleSpecimen1IntakePos), new Point(cycleSPecimenBehindPos)))
                .setConstantHeadingInterpolation(cycleSPecimenBehindPos.getHeading())
                .build();

        mergedPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(cycleSpecimen1LineupPos), new Point(cycleSpecimen1IntakePos)))
                .setConstantHeadingInterpolation(cycleSpecimen1IntakePos.getHeading())
                .addPath(new BezierLine(new Point(cycleSpecimen1LineupPos), new Point(cycleSpecimen1IntakePos)))
                .setConstantHeadingInterpolation(cycleSpecimen1IntakePos.getHeading())
                .build();
        cycleSpecimenObs1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(cycleSPecimenBehindPos), new Point(cycleSpecimenObs1Pos)))
                .setConstantHeadingInterpolation(cycleSpecimenObs1Pos.getHeading())
                .build();
        cycleSpecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(cycleSpecimenObs1Pos), new Point(cycleSPecimenBehindPos)))
                .setConstantHeadingInterpolation(cycleSPecimenBehindPos.getHeading())
                .build();
        cycleSpecimen2Behind = follower.pathBuilder()
                .addPath(new BezierLine(new Point(cycleSPecimenBehindPos), new Point(cycleSpecimen2BehindPos)))
                .setConstantHeadingInterpolation(cycleSpecimen2BehindPos.getHeading())
                .build();
        cycleSpecimen3Behind = follower.pathBuilder()
                .addPath(new BezierLine(new Point(cycleSpecimen1IntakePos), new Point(specimen3BehindPos)))
                .setConstantHeadingInterpolation(cycleSpecimen2BehindPos.getHeading())
                .build();

        cycleSpecimenObs2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(cycleSpecimen2BehindPos), new Point(cycleSpecimenObs2Pos)))
                .setConstantHeadingInterpolation(cycleSpecimenObs2Pos.getHeading())
                .build();
        backUpFromObs = follower.pathBuilder()
                .addPath(new BezierLine(new Point(cycleSpecimenObs1Pos), new Point(backUpFromObsPos)))
                .setConstantHeadingInterpolation(backUpFromObsPos.getHeading())
                .build();
        turnObs1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(backUpFromObsPos), new Point(turnObs1Pos)))
                .setConstantHeadingInterpolation(turnObs1Pos.getHeading())
                .build();
        lineUpGrabSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(backUpFromObsPos), new Point(lineUpGrabSpecimen)))
                .setConstantHeadingInterpolation(lineUpGrabSpecimen.getHeading())
                .build();
        specimenCycleLineUpPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenScorePos), new Point(specimenCycleLineUp)))
                .setConstantHeadingInterpolation(specimenCycleLineUp.getHeading())
                .build();
        grabSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(turnObs1Pos), new Point(grabSpecimenPos)))
                .setConstantHeadingInterpolation(grabSpecimenPos.getHeading())
                .build();

        cheeseAuto = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimen1ScorePos), new Point(bezPoint2), new Point(bezPoint), new Point(grabSpecimenPos)))
                .setConstantHeadingInterpolation(grabSpecimenPos.getHeading())
                .build();
        cheeseCycleAuto = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenCycleLineUp), new Point(specimen2ScorePos)))
                .setConstantHeadingInterpolation(specimen2ScorePos.getHeading())
                .build();
        lineUpScoreSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabSpecimenPos), new Point(specimenCycleLineUp)))
                .setConstantHeadingInterpolation(specimenCycleLineUp.getHeading())
                .build();
        scoreSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenCycleLineUp), new Point(specimen1ScorePos)))
                .setConstantHeadingInterpolation(specimen1ScorePos.getHeading())
                .build();
//        specimenPos2Path = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(grabSpecimen1), new Point(specimenPos2)))
//                .setConstantHeadingInterpolation(specimenPos2.getHeading())
//                .build();
//        specimenScorePos2Path = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(specimenPos2), new Point(specimenScorePos2)))
//                .setConstantHeadingInterpolation(specimenScorePos2.getHeading())
//                .build();
        grabSpecimen2Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(cycleSpecimenObs3Pos), new Point(grabSpecimen1)))
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
        lineUpGrabSpecimenPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(cycleSpecimenObs2Pos), new Point(lineUpGrabSpecimen)))
                .setConstantHeadingInterpolation(lineUpGrabSpecimen.getHeading())
                .build();
        specimenCycleLineUpPath2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenScorePos2), new Point(specimenCycleLineUp)))
                .setConstantHeadingInterpolation(specimenCycleLineUp.getHeading())
                .build();
        cycleSpecimen3Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenCycleLineUp), new Point(cycleSpecimen3Pos)))
                .setConstantHeadingInterpolation(cycleSpecimen3Pos.getHeading())
                .build();
        cycleSpecimenObs3Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen3BehindPos), new Point(cycleSpecimenObs3Pos)))
                .setConstantHeadingInterpolation(cycleSpecimenObs3Pos.getHeading())
                .build();
        lineUpGrabSpecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenCycleLineUp), new Point(lineUpGrabSpecimen)))
                .setConstantHeadingInterpolation(grabSpecimen1.getHeading())
                .build();






    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                Actions.runBlocking(endEffector.closeClaw);
                Actions.runBlocking(endEffector.diffyInit);
                setPathState(2);
                break;
//            case 1:
//                if (!follower.isBusy()) {
//                    //Actions.runBlocking(preSpecimenScore());
////                    Actions.runBlocking(arm.autoArmPreSpecimen);
//                    //follower.followPath(specimenPath);
//                    setPathState(2);
//                }
//                break;
            case 2:
                if (!follower.isBusy()) {
                    Actions.runBlocking(endEffector.closeClaw);
                    Actions.runBlocking(endEffector.autoPreSpecimen);
                    Actions.runBlocking(arm.autoArmPreSpecimen);
                    Actions.runBlocking(new SleepAction(.25));
                    //Actions.runBlocking(specimenScore());
//                    Actions.runBlocking(endEffector.openClawAuto);
                    follower.followPath(scorePath);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(specimenCycleLineUpPath);
                    Actions.runBlocking(endEffector.openClawAuto);
                    //follower.followPath(parkPath);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    Actions.runBlocking(arm.armMax);
                    Actions.runBlocking(endEffector.diffyIdle);
                    //Actions.runBlocking(endEffector.diffyIdle);
                    follower.followPath(cycleSpecimen1Lineup);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    Actions.runBlocking(endEffector.diffyObs);
                    follower.followPath(cycleSpecimen1);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(specimen1Behind);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(cycleSpecimenObs1);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()){
                    follower.followPath(backUpFromObs);
//                follower.followPath(cycleSpecimen3Path);
//                follower.followPath(cycleSpecimenObs3Path);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    Actions.runBlocking(arm.armObservation);
                    Actions.runBlocking(endEffector.openClawAuto);
                    Actions.runBlocking(endEffector.diffyObs);
                    follower.followPath(turnObs1);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(grabSpecimen);
                    // setPathState(112);
                }
                if (follower.getVelocity().getXComponent() == 0 && pathTimer.getElapsedTimeSeconds() > 0.25) {
                    setPathState(112);
                }
                break;
            case 112:
                Actions.runBlocking(endEffector.closeClaw);
                setPathState(15);
                break;
            case 15:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2) {
                    Actions.runBlocking(endEffector.closeClaw);
                    Actions.runBlocking(new SleepAction(.25));
//                    Actions.runBlocking(preSpecimenScore());
////                    Actions.runBlocking(arm.autoArmPreSpecimen);
                    follower.followPath(lineUpScoreSpecimen1);
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    Actions.runBlocking(endEffector.autoPreSpecimen);
                    Actions.runBlocking(arm.autoArmSpecimen2);
                    follower.followPath(scoreSpecimen1);
//                    Actions.runBlocking(endEffector.openClawAuto);
                    setPathState(17);
                }
                break;
            case 17:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 1) {
                    Actions.runBlocking(endEffector.openClawAuto);
                    setPathState(18);
                }
                break;
            case 18:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(cheeseAuto);
                    Actions.runBlocking(new SleepAction(0.5));
                    Actions.runBlocking(arm.armObservation);
                    Actions.runBlocking(endEffector.diffyObs);
                    setPathState(21);
                }
                break;
            case 21:
                if (!follower.isBusy()) {
                    Actions.runBlocking(endEffector.closeClaw);
                    Actions.runBlocking(new SleepAction(0.5));
                    setPathState(19);
                }
                break;

            case 19:
                if (!follower.isBusy()) {
                    follower.followPath(lineUpScoreSpecimen1);
                    setPathState(20);
                }
                break;
            case 20:
                if (!follower.isBusy()) {
                    Actions.runBlocking(endEffector.autoPreSpecimen);
                    Actions.runBlocking(arm.autoArmSpecimen2);
                    follower.followPath(cheeseCycleAuto);
                    setPathState(69);
                }
                break;
            case 69:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 1) {
                    Actions.runBlocking(endEffector.openClawAuto);
                }
                setPathState(300);
                break;
            case 300 :
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
        telemetry.addData("Arm Pos", arm.getCurrentPosition());
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
        endEffector.initPosition();
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
                // endEffector.autoSpecimen,
                // arm.autoArmSpecimen,
                new SleepAction(1)
                // new SleepAction(3)
        );
    }

}