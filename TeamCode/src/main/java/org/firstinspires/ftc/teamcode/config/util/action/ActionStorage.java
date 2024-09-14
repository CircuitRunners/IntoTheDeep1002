package org.firstinspires.ftc.teamcode.config.util.action;

public class ActionStorage {
    public RunAction preparePurple = new RunAction(this::preparePurple);
    public RunAction scorePurple = new RunAction(this::scorePurple);
    public RunAction prepareYellow = new RunAction(this::prepareYellow);
    public RunAction scoreYellow = new RunAction(this::scoreYellow);
    public RunAction preparePark = new RunAction(this::preparePark);
    public RunAction finishPark = new RunAction(this::finishPark);
    //public SequentialAction auto = new SequentialAction(new ParallelAction(preparePurple, new SleepAction(1)), scorePurple, prepareYellow, scoreYellow, preparePark, finishPark);

    public void preparePurple() {}
    public void scorePurple() {}
    public void prepareYellow() {}
    public void scoreYellow() {}
    public void preparePark() {}
    public void finishPark() {}
}