package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.Subsystems.LimelightCenterSubsystem;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

@TeleOp
public class LimelightTest extends OpMode {

    private RobotHardware robot;
    private LimelightCenterSubsystem limelight;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.init();

        limelight = new LimelightCenterSubsystem(robot);

        // Configure horizontal offsets (inches) from the center of the 4 AprilTags.
        // Adjust these IDs and offsets (in inches) to match your game target layout!
        limelight.setTagXOffset(1, -12.0); // 12 inches left of center
        limelight.setTagXOffset(2,  -4.0); // 4 inches left of center
        limelight.setTagXOffset(3,  +4.0); // 4 inches right of center
        limelight.setTagXOffset(4, +12.0); // 12 inches right of center

        limelight.init();
    }

    @Override
    public void loop() {
        if (robot.limelightAdjServo != null) {
            robot.limelightAdjServo.setPosition(0.65);
        }

        limelight.run();

        telemetry.addData("isCenterValid", limelight.isCenterValid());
        telemetry.addData("txToCenter", limelight.getTxToCenter());
        telemetry.addData("rawTx", limelight.getRawTx());
        telemetry.addData("distanceInches", limelight.getDistanceToCenterInches());
        telemetry.addData("tagsDetected", limelight.getTagDetected());
        telemetry.update();
    }
}

