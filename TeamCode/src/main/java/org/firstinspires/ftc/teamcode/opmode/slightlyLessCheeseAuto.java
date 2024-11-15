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
import org.firstinspires.ftc.teamcode.config.util.action.Actions;
import org.firstinspires.ftc.teamcode.config.util.action.SleepAction;

@Autonomous (name = "Gortimus Maximus Cheese Auto")
public class slightlyLessCheeseAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private Arm arm;
    private EndEffector endEffector;

    // Define key poses (x, y, and heading values not set)
    private Pose startPos = new Pose(10, 43, Math.toRadians(180));
    private Pose controlPointSample1Pos = new Pose(90, 24, Math.toRadians(180));
    private Pose controlPointSampleone2Pos = new Pose(93, 22, Math.toRadians(180));
    private Pose behindSample1Pos = new Pose(50, 22, Math.toRadians(180));
    private Pose ObsSample1Pos = new Pose(12, 22, Math.toRadians(180));
    private Pose controlPointSample2Pos = new Pose(85, 16, Math.toRadians(0));
    private Pose behindSample2Pos = new Pose(50, 8, Math.toRadians(180));
    private Pose ObsSample2Pos = new Pose(12, 10, Math.toRadians(180));
    private Pose backPos = new Pose(20, 19, Math.toRadians(0));
    private Pose intakeObsPos = new Pose(8.8, 19, Math.toRadians(180));
//    private Pose hangSpecimen1Pos = new Pose(64, 33, Math.toRadians(180));
//    private Pose hangSpecimen2Pos = new Pose(64, 33, Math.toRadians(180));
//    private Pose hangSpecimen3Pos = new Pose(64, 33, Math.toRadians(180));
//    private Pose hangSpecimen4Pos = new Pose(64, 33, Math.toRadians(180));
//    private Pose parkPos = new Pose(64, 33, Math.toRadians(180));

    private PathChain behindSample1Path, pushSample1Path, behindSample2Path, pushSample2Path, backPath, intakeObs1Path, hangSpecimen1Path, intakeObs2Path, hangSpecimen2Path, intakeObs3Path, hangSpecimen3Path, intakeObs4Path, hangSpecimen4Path, parkPath;
    public void buildPaths() {
        behindSample1Path = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPos), new Point(controlPointSample1Pos),new Point(behindSample1Pos)))
                .setConstantHeadingInterpolation(behindSample1Pos.getHeading())
                .build();
        pushSample1Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(behindSample1Pos), new Point(ObsSample1Pos)))
                .setConstantHeadingInterpolation(ObsSample1Pos.getHeading())
                .build();
        behindSample2Path = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(ObsSample1Pos), new Point(controlPointSample2Pos), new Point(behindSample2Pos)))
                .setConstantHeadingInterpolation(behindSample2Pos.getHeading())
                .build();

        pushSample2Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(behindSample2Pos), new Point(ObsSample2Pos)))
                .setConstantHeadingInterpolation(ObsSample2Pos.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                Actions.runBlocking(endEffector.closeClaw);
                Actions.runBlocking(endEffector.diffyInit);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(behindSample1Path);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(pushSample1Path);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(behindSample2Path);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(pushSample2Path);
                    setPathState(100);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(backPath);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(intakeObs1Path);
                    setPathState(100);
                }
                break;
            case 100:
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
        follower.setStartingPose(startPos);
        arm = new Arm(hardwareMap);
        endEffector = new EndEffector(hardwareMap);
        endEffector.closeClaw();
        endEffector.initPosition();
        follower.setMaxPower(1);
        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

}
