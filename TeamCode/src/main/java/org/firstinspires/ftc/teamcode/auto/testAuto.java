package org.firstinspires.ftc.teamcode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous (name = "Test Auto Path")
public class testAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;


    private Pose startPoint = new Pose(9.757, 84.98,0);
    private Pose chamber = new Pose(36.784,77.275,0);
    private Pose spikeMark1 = new Pose(53.608, 114.059,0);
    private Pose net1 = new Pose(14.828,129.743,0);
    private Pose spikeMark2 = new Pose(69.58,123.75,0);
    private Pose net2 = new Pose(17.96,133.16,0);
    private Pose spikeMark3 = new Pose(66.725,134.305,0);
    private Pose net3 = new Pose(18,136.871,0);
    private Pose observation = new Pose(18,18.250,0);
    @Override
    public void loop(){

    }

    @Override
    public void init(){
        pathTimer = new Timer();
        follower = new Follower(hardwareMap);
        follower.setPose(startPoint);

    }

    @Override
    public void start(){
        pathTimer.resetTimer();

    }

}
