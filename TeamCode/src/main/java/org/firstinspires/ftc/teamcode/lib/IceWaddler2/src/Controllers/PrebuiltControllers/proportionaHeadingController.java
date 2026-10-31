package org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Controllers.PrebuiltControllers;

import static org.firstinspires.ftc.teamcode.lib.IceWaddler2.IWConfig.headingKP;
import static org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Units.Unit.perSecond;

import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Controllers.HeadingController;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;

public class proportionaHeadingController implements HeadingController {
    public proportionaHeadingController(double kP){}

    @Override
    public Scalar getCorrection(NormalizedAngle error) {
        return error.toScalar().multiply(new Scalar(-headingKP,perSecond));
    }
}