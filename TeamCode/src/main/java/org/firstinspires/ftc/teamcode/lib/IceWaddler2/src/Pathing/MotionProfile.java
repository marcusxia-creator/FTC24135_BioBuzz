package org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Pathing;

import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Scalar;

public interface MotionProfile {
    void init(Scalar startVel, Scalar endVel, Scalar totalDistance);

    Scalar getVel(double completion);
}