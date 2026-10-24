package org.firstinspires.ftc.teamcode.lib.GamepadDriver.PrebuiltInputs.AnalogInputs;

import static java.util.Arrays.stream;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.GamepadDriver.GamepadInput;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Velocity;

import java.util.LinkedList;
import java.util.Queue;

public class SmoothedVel implements GamepadInput<Velocity>{
    GamepadInput<Velocity> rawVel;
    Queue<Velocity> velocities=new LinkedList<>();
    int ticks;

    ElapsedTime tickTimer;

    public SmoothedVel(GamepadInput<Velocity>rawVel, int ticks){
        this.rawVel=rawVel;
        this.ticks=ticks;
    }

    @Override
    public void update() {
        rawVel.update();
        velocities.add(rawVel.get());

        if(velocities.size()>ticks){
            velocities.poll();
        }
    }

    @Override
    public Velocity get() {
        return velocities.stream().reduce(Velocity.zero,Velocity::add).div(velocities.size());
    }
}