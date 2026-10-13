package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotHardware;
import static org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotActionConfig.*;

@TeleOp
public class Drive_N_Turn extends OpMode {

    private RobotHardware robot;

    private Pose2D[] cells;
    private int selectedCell = 0;

    private final double SWITCH_MARGIN = 6.0; //inch
    private final double STICK_DEADBAND = 0.05;
    private final double HEADING_DEADBAND = 1.5;
    private final double KP_TURN = 0.01;
    private final double MAX_TURN_POWER = 0.5;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.initPinpoint();

        cells = new Pose2D[] {
                blueAllianceCell1,
                blueAllianceCell2,
                redAllianceCell2,
                redAllianceCell1
        };
    }

    @Override
    public void loop() {
        robot.pinpoint.update();

        selectedCell = chooseCell(
                robot.pinpoint.getPosX(DistanceUnit.INCH),
                robot.pinpoint.getPosY(DistanceUnit.INCH),
                selectedCell
        );

        Pose2D targetCell = cells[selectedCell];

        double targetCellX = targetCell.getX(DistanceUnit.INCH);
        double targetCellY = targetCell.getY(DistanceUnit.INCH);

        double targetHeading = Math.toDegrees(
                Math.atan2(
                        targetCellY - robot.pinpoint.getPosY(DistanceUnit.INCH),
                        targetCellX - robot.pinpoint.getPosX(DistanceUnit.INCH)
                )
        );

        double error = wrapDegrees (
            targetHeading - robot.pinpoint.getHeading(AngleUnit.DEGREES)
        );

        double forward = -gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;

        if (Math.abs(forward) < STICK_DEADBAND) {
            forward = 0;
        }
        if (Math.abs(strafe) < STICK_DEADBAND) {
            strafe = 0;
        }

        double turn;

        if (gamepad1.dpad_left) {
            turn = Math.abs(error) < HEADING_DEADBAND ? 0.0:
                    Range.clip(KP_TURN * error, -MAX_TURN_POWER, MAX_TURN_POWER);
        }
        else {
            turn = gamepad1.left_stick_x;
        }

        double frontLeftPower = forward + strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backLeftPower = forward - strafe + turn;
        double backRightPower = forward + strafe - turn;

        double maxPower = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
                )
        );

        robot.frontLeftMotor.setPower(frontLeftPower / maxPower);
        robot.frontRightMotor.setPower(frontRightPower / maxPower);
        robot.backLeftMotor.setPower(backLeftPower / maxPower);
        robot.backRightMotor.setPower(backRightPower / maxPower);

        telemetry.addData("Auto Facing", gamepad1.dpad_left);
        telemetry.addData("Selected Cell", selectedCell + 1);
        telemetry.addData("x", robot.pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("y", robot.pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("heading", robot.pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }

    private int chooseCell(double x, double y, int current) {
        int closest = current;
        double currentDistance = distanceToCell(x, y, cells[current]);
        double bestDistance = currentDistance;

        for (int i = 0; i < cells.length; i++) {
            double distance = distanceToCell(x, y, cells[i]);

            if (distance < bestDistance) {
                bestDistance = distance;
                closest = i;
            }
        }

        if (closest != current && bestDistance + SWITCH_MARGIN < currentDistance) {
            return closest;
        }

        return current;
    }

    private double distanceToCell (double x, double y, Pose2D cell) {
        double dx = cell.getX(DistanceUnit.INCH) - x;
        double dy = cell.getY(DistanceUnit.INCH) - y;

        return Math.hypot(dx, dy);
    }

    private double wrapDegrees(double angle) {
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }
}
