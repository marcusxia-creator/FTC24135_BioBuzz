package org.firstinspires.ftc.teamcode.src.Subsystems;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.blueAllianceCell1;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.blueAllianceCell2;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.redAllianceCell1;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.redAllianceCell2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

@Config
public class RobotDriveSubsystem extends SubsystemBase {
    double lastForward;
    double lastStrafe;
    double lastTurn;
    private RobotHardware robot;

    ///DRIVE_N_TURN
    private Pose2D[] cells;
    private int selectedCell = 0;

    private final double SWITCH_MARGIN = 6.0; //inch
    private final double STICK_DEADBAND = 0.05;
    public static double HEADING_DEADBAND = 1.5;
    public static double KP_TURN = 0.01;
    public static double MAX_TURN_POWER = 0.5;

    private double FLPower = 0;
    private double FRPower = 0;
    private double BLPower = 0;
    private double BRPower = 0;

    public RobotDriveSubsystem (RobotHardware robot) {
        this.robot = robot;
        this.cells = new Pose2D[] {
                blueAllianceCell1,
                blueAllianceCell2,
                redAllianceCell2,
                redAllianceCell1
        };
    }

    @Override
    public void periodic() {
        robot.odo.update();
    }

    public void mecanumDrive (double forward, double strafe, double turn, boolean ps){
        double lerp =0.15;

        if (Math.abs(forward) < STICK_DEADBAND) forward = 0.0;
        if (Math.abs(strafe) < STICK_DEADBAND) strafe = 0.0;

        selectedCell = chooseCell(
                robot.odo.getPosX(DistanceUnit.INCH),
                robot.odo.getPosY(DistanceUnit.INCH),
                selectedCell
        );

        Pose2D targetCell = cells[selectedCell];

        double error = wrapDegrees(
                getTargetHeading(robot.odo.getPosX(DistanceUnit.INCH), robot.odo.getPosY(DistanceUnit.INCH), targetCell)
                        - robot.odo.getHeading(UnnormalizedAngleUnit.DEGREES)
        );

        if (ps) {
            lastForward = forward;
            lastStrafe = strafe;

            if (Math.abs(error) < HEADING_DEADBAND) {
                turn = 0;
            }
            else {
                turn = Range.clip(-KP_TURN * error, -MAX_TURN_POWER, MAX_TURN_POWER);
            }
            lastTurn = turn;

            FLPower = forward + strafe + turn;
            FRPower = forward - strafe - turn;
            BLPower = forward - strafe + turn;
            BRPower = forward + strafe - turn;
        }
        else {
            lastForward = lastForward + lerp*(forward - lastForward);
            lastStrafe = lastStrafe + lerp * (strafe - lastStrafe);
            lastTurn = lastTurn + lerp * (turn - lastTurn);

            FLPower = lastForward + lastStrafe + lastTurn;
            FRPower = lastForward - lastStrafe - lastTurn;
            BLPower = lastForward - lastStrafe + lastTurn;
            BRPower = lastForward + lastStrafe - lastTurn;
        }

        double max = Math.max(Math.abs(FLPower), Math.max(Math.abs(FRPower),
                Math.max(Math.abs(BLPower), Math.abs(BRPower))));

        if (max > 1.0) {
            FLPower /= max;
            FRPower /= max;
            BLPower /= max;
            BRPower /= max;
        }

        robot.frontLeftMotor.setPower(FLPower);
        robot.frontRightMotor.setPower(FRPower);
        robot.backLeftMotor.setPower(BLPower);
        robot.backRightMotor.setPower(BRPower);
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

    private double getTargetHeading(double x, double y, Pose2D targetCell) {
        double dx = targetCell.getX(DistanceUnit.INCH) - x;
        double dy = targetCell.getY(DistanceUnit.INCH) - y;
        return Math.toDegrees(Math.atan2(dy, dx));// + 180;
    }

    private double wrapDegrees(double angle) {
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }

}
