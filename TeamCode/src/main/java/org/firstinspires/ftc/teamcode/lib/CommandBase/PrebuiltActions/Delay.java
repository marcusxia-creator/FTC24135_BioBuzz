package org.firstinspires.ftc.teamcode.lib.CommandBase.PrebuiltActions;

import static org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Units.Unit.s;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.CommandBase.Action;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Scalar;

public class Delay implements Action {
    double delaySeconds;
    ElapsedTime timer;
    public Delay(double delaySeconds){
        this.delaySeconds=delaySeconds;
    }

    public Delay(Scalar delay){
        this.delaySeconds=delay.getValue(s);
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
