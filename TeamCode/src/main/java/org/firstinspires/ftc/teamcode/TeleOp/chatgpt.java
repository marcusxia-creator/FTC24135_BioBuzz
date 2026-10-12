
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "BIOBUZZ Orbit Test", group = "Test")
public class chatgpt extends LinearOpMode {

    private DcMotor fl, fr, bl, br;
    private GoBildaPinpointDriver pinpoint;

    // ----------------------------------------------------
    // FOUR PLACEHOLDER HIVE CELL POSITIONS
    // Coordinates are inches in the Pinpoint field frame.
    // Replace these with measured BIOBUZZ cell positions.
    // The heading component is unused for cell locations.
    // ----------------------------------------------------

    private final Pose2D cell1 = new Pose2D(
            DistanceUnit.INCH, -48, 48,
            AngleUnit.DEGREES, 0);

    private final Pose2D cell2 = new Pose2D(
            DistanceUnit.INCH, 48, 48,
            AngleUnit.DEGREES, 0);

    private final Pose2D cell3 = new Pose2D(
            DistanceUnit.INCH, -48, -48,
            AngleUnit.DEGREES, 0);

    private final Pose2D cell4 = new Pose2D(
            DistanceUnit.INCH, 48, -48,
            AngleUnit.DEGREES, 0);

    private Pose2D[] cells;

    // Tuning
    private static final double KP_TURN = 0.018;
    private static final double MAX_TURN = 0.35;
    private static final double HEADING_DEADBAND = 2.0;
    private static final double SWITCH_MARGIN_IN = 6.0;
    private static final double STICK_DEADBAND = 0.05;

    private boolean autoFacing = true;
    private boolean previousGuide = false;
    private int selectedCell = 0;

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotor.class, "FL_Motor");
        fr = hardwareMap.get(DcMotor.class, "FR_Motor");
        bl = hardwareMap.get(DcMotor.class, "BL_motor");
        br = hardwareMap.get(DcMotor.class, "BR_motor");

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class, "pinpoint");

        cells = new Pose2D[] {
                cell1, cell2, cell3, cell4
        };

        // Match these directions to your robot.
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMPORTANT:
        // Configure Pinpoint pod resolution, pod offsets, and
        // encoder directions for your actual installation.
        // Do not assume the example cell coordinates are correct.

        telemetry.addLine("BIOBUZZ Orbit Test");
        telemetry.addLine("Guide: toggle auto-facing");
        telemetry.addLine("Left stick Y: radial movement");
        telemetry.addLine("Left stick X: orbit movement");
        telemetry.addLine("Right stick X: manual rotation if auto is off");
        telemetry.addLine("Cell coordinates are placeholders!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Update localization every loop.
            pinpoint.update();

            Pose2D robotPose = pinpoint.getPosition();

            double robotX = robotPose.getX(DistanceUnit.INCH);
            double robotY = robotPose.getY(DistanceUnit.INCH);
            double heading = robotPose.getHeading(
                    AngleUnit.DEGREES);

            // Toggle automatic facing on a fresh Guide press.
            boolean guidePressed = gamepad1.guide;

            if (guidePressed && !previousGuide) {
                autoFacing = !autoFacing;
            }

            previousGuide = guidePressed;

            // Choose a target using Pinpoint position.
            // Hysteresis prevents rapid switching near boundaries.
            selectedCell = chooseCell(
                    robotX, robotY, selectedCell);

            Pose2D target = cells[selectedCell];

            double targetX = target.getX(DistanceUnit.INCH);
            double targetY = target.getY(DistanceUnit.INCH);

            // Direction from the robot to the target cell.
            double targetHeading = Math.toDegrees(
                    Math.atan2(
                            targetY - robotY,
                            targetX - robotX));

            double error = wrapDegrees(
                    targetHeading - heading);

            // Driver translation inputs.
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;

            if (Math.abs(forward) < STICK_DEADBAND) {
                forward = 0;
            }

            if (Math.abs(strafe) < STICK_DEADBAND) {
                strafe = 0;
            }

            // Auto-facing: hold aim at the selected cell.
            // With the robot facing the cell, local strafe
            // becomes tangential motion around that cell.
            double turn;

            if (autoFacing) {
                turn = Math.abs(error) < HEADING_DEADBAND
                        ? 0.0
                        : Range.clip(
                        KP_TURN * error,
                        -MAX_TURN,
                        MAX_TURN);
            } else {
                turn = -gamepad1.right_stick_x;
            }

            // Robot-centric mecanum mixing.
            // Forward/backward = radial component.
            // Strafe left/right = tangential component.
            double pFL = forward + strafe + turn;
            double pFR = forward - strafe - turn;
            double pBL = forward - strafe + turn;
            double pBR = forward + strafe - turn;

            // Normalize to keep powers within [-1, 1].
            double maxPower = Math.max(
                    1.0,
                    Math.max(
                            Math.max(Math.abs(pFL), Math.abs(pFR)),
                            Math.max(Math.abs(pBL), Math.abs(pBR))
                    )
            );

            fl.setPower(pFL / maxPower);
            fr.setPower(pFR / maxPower);
            bl.setPower(pBL / maxPower);
            br.setPower(pBR / maxPower);

            telemetry.addData("Auto Facing", autoFacing);
            telemetry.addData("Selected Cell", selectedCell + 1);
            telemetry.addData("Robot X", "%.1f in", robotX);
            telemetry.addData("Robot Y", "%.1f in", robotY);
            telemetry.addData("Heading", "%.1f deg", heading);
            telemetry.addData("Target Heading",
                    "%.1f deg", targetHeading);
            telemetry.addData("Heading Error", "%.1f deg", error);
            telemetry.addData("Target X", "%.1f in", targetX);
            telemetry.addData("Target Y", "%.1f in", targetY);
            telemetry.update();
        }

        stopMotors();
    }

    private int chooseCell(double x, double y, int current) {

        int closest = current;
        double currentDistance = distanceToCell(x, y, cells[current]);
        double bestDistance = currentDistance;

        for (int i = 0; i < cells.length; i++) {
            double d = distanceToCell(x, y, cells[i]);

            if (d < bestDistance) {
                bestDistance = d;
                closest = i;
            }
        }

        // Switch only when the new cell is sufficiently closer.
        if (closest != current
                && bestDistance + SWITCH_MARGIN_IN
                < currentDistance) {
            return closest;
        }

        return current;
    }

    private double distanceToCell(
            double x, double y, Pose2D cell) {

        double dx = cell.getX(DistanceUnit.INCH) - x;
        double dy = cell.getY(DistanceUnit.INCH) - y;

        return Math.hypot(dx, dy);
    }

    private double wrapDegrees(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    private void stopMotors() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
