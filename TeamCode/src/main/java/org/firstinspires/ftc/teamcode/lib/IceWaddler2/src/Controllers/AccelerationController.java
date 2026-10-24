package org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Controllers;

import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;

public interface AccelerationController {
    Acceleration getCorrection(Velocity error);
}
