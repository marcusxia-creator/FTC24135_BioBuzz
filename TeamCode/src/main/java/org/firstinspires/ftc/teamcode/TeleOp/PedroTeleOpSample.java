package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.JoystickDriveProcessor;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Pedro TeleOp Sample")
public class PedroTeleOpSample extends CommandOpMode {
    private Follower follower;
    private final ElapsedTime driveTimer = new ElapsedTime();
    // Starter settings: deadzone, exponent, acceleration, deceleration.
    // Translation: 0 -> full power in 0.5 s; full -> zero in 0.25 s.
    private final JoystickDriveProcessor joystick = new JoystickDriveProcessor(
            new JoystickDriveProcessor.JoystickResponseConfig(0.05, 2.0, 2.0, 4.0),
            new JoystickDriveProcessor.JoystickResponseConfig(0.05, 2.0, 1.5, 3.0));

    @Override
    public void initialize() {
        super.reset();
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void preRun() {
        joystick.reset();
        driveTimer.reset();
    }

    @Override
    public void run() {
        super.run();
        // Robot-centric: positive forward, left strafe, and counterclockwise turn.
        double dt = driveTimer.seconds();
        driveTimer.reset();
        JoystickDriveProcessor.Output drive = joystick.update(
                -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, dt);
        follower.manual(drive.forward, drive.strafe, drive.turn);
        follower.update();

        Pose pose = follower.pose();
        telemetry.addData("X (in)", pose.x());
        telemetry.addData("Y (in)", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.update();
    }

    @Override
    public void reset() {
        joystick.reset();
        if (follower != null) {
            follower.stop();
        }
        super.reset();
    }
}


