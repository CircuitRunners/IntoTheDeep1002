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
    private Pose pointOne = new Pose(36.784,77.275,0);
    private Pose pointTwo = new Pose(53.608, 114.059,0);
    private Pose pointThree = new Pose(14.828,129.743,0);


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
