package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.Subsystems.Limelight2DCenterSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.LimelightCenterSubsystem;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

@TeleOp
public class LimelightTest extends OpMode {

    private RobotHardware robot;
    private Limelight2DCenterSubsystem limelight;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.init();

        limelight = new Limelight2DCenterSubsystem(robot);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Configure horizontal offsets (inches) from the center of the 4 AprilTags.
        // Adjust these IDs and offsets (in inches) to match your game target layout!
        limelight.setTagXOffset(38, -6.5); // 12 inches left of center
        limelight.setTagXOffset(39,  -2.75); // 4 inches left of center
        limelight.setTagXOffset(40,  2.75); // 4 inches right of center
        limelight.setTagXOffset(41, 6.5); // 12 inches right of center

        limelight.init(false);
    }

    @Override
    public void loop() {

        /**
        if (robot.limelightAdjServo != null) {
            robot.limelightAdjServo.setPosition(0.65);
        }
        */

        limelight.run();

        telemetry.addData("isCenterValid", limelight.isCenterValid());
        telemetry.addData("txToCenter", limelight.getTxToCenter());
        telemetry.addData("rawTx", limelight.getRawTx());
        telemetry.addData("distanceInches", limelight.getDistanceToCenterInches());
        telemetry.addData("tagsDetected", limelight.getTagDetected());
        telemetry.update();
    }
}

