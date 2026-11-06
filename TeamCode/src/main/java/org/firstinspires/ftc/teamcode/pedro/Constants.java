package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** Pedro 3.0.2 configuration shared by TeleOp and the tuning procedures. */
public final class Constants {
    private Constants() { }

    public static final MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FL_Motor");
        c.backLeftName.set("BL_Motor");
        c.frontRightName.set("FR_Motor");
        c.backRightName.set("BR_Motor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {
        // VERIFY: exact device name in the Robot Controller hardware configuration.
        c.name.set("pinpoint");
        // VERIFY: change to goBILDA_SWINGARM_POD or configure ticksPerUnit for custom pods.
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // TUNE: forward motion must increase X; leftward motion must increase Y.
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        // TUNE: signed offsets from the center of rotation, in inches.
        // Replace these zero placeholders with the Pinpoint tuner's results.
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(0.0);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        // Uses Pinpoint's built-in IMU. Keep the robot still during initialization.
        c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
    });

    /**
     * MANUAL-DRIVE STARTER ONLY. These are not measured path-following constants.
     * Replace this entire configuration with the Foresight tuner's generated output
     * before using follow() or hold(). Manual driving bypasses this algorithm.
     */
    public static final ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.headingFeedback.set(Controller.zero);
        c.forwardTranslational.set(Controller.zero);
        c.strafeTranslational.set(Controller.zero);
        c.brake.set(Controller.zero);
        c.coast.set(Controller.zero);
        c.linearBrakeCoefficients.set(Matrix.diag(1.0, 1.0));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0, 0.0));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(1.0, 0.0));
        // Positive placeholders required by Pedro; units are inches and seconds.
        c.maxAchievableForwardVelocity.set(1.0);
        c.maxAchievableStrafeVelocity.set(1.0);
        c.naturalForwardDeceleration.set(1.0);
        c.naturalStrafeDeceleration.set(1.0);
    });

    /** Create once during initialization, after any other drivetrain hardware setup. */
    public static Follower createFollower(HardwareMap hardwareMap) {
        for (String name : new String[] {
                drivetrainConfig.frontLeftName.get(), drivetrainConfig.backLeftName.get(),
                drivetrainConfig.frontRightName.get(), drivetrainConfig.backRightName.get()}) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        return new Follower(new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig), new Foresight(foresightConfig));
    }

}
