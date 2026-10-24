package org.firstinspires.ftc.teamcode.lib.CommandBase.PrebuiltActions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.CommandBase.Action;

public class Delay implements Action {
    double delaySeconds;
    ElapsedTime timer;
    public Delay(double delaySeconds){
        this.delaySeconds=delaySeconds;
    }

    @Override
    public void init() {
        timer=new ElapsedTime();
        timer.reset();
    }

    @Override
    public void loop() {}

    @Override
    public void shutdown() {
        timer=null;
    }

    @Override
    public boolean finished() {
        return timer.seconds()>=delaySeconds;
    }
}
