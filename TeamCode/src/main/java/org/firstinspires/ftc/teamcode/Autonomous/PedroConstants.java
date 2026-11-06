package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Compatibility entry point; configuration now lives in pedro.Constants (Pedro 3.0.2). */
public final class PedroConstants {
    private PedroConstants() { }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return Constants.createFollower(hardwareMap);
    }
}
