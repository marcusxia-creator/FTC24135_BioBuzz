package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.LUT;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

import java.util.Optional;

@Config
public class PowerCalculator {

    private final PIDController pidController;
    private int zone = 0;
    private double distance = 0;

    private final LUT<Integer, Integer> targetRPM = new LUT<Integer, Integer>() {{
        add(1, RPM1);
        add(0, RPM0);
    }};

    /// PID
    public static double kP = 2;
    public static double kI = 0.0;
    public static double kD = 0.02;

    /// Feedforward
    public static double kSShooter = 0.03;  // static friction (small bump)
    public static double kVShooter = 1.0;   // scale from targetNorm to power (roughly 1.0 if perfect)

    /// Switch Hysteresis Margin in inches to prevent rapid flickering between target cells
    public static double cellSwitchMargin = 4.0;

    private Pose2D[] actualGoal;
    private Pose2D currentTargetCell = null;

    private int rpmTarget;
    private double rpmMeasured;

    public PowerCalculator() {
        this.pidController = new PIDController(kP, kI, kD);
    }

    /**
     * Sets the target cells according to alliance color.
     * Resets currentTargetCell so the closest cell is re-evaluated.
     *
     * @param isRedAlliance true for Red Alliance, false for Blue Alliance
     */
    public void setAlliance(boolean isRedAlliance) {
        if (isRedAlliance) {
            actualGoal = new Pose2D[] {redAllianceCell1, redAllianceCell2};
        } else {
            actualGoal = new Pose2D[] {blueAllianceCell1, blueAllianceCell2};
        }
    }

    /**
     * Gets the currently locked target cell.
     */
    public Pose2D getCurrentTargetCell() {
        return currentTargetCell;
    }


    /**
     * Finds and returns the closest cell in actualGoal to the robot's (x, y) coordinates in inches.
     *
     * @param robotX Robot X coordinate in inches
     * @param robotY Robot Y coordinate in inches
     * @return Pose2D of the target cell
     */
    public Pose2D getClosestCell(double robotX, double robotY) {
        if (actualGoal == null || actualGoal.length == 0) {
            return null;
        }

        // If only one cell exists for this alliance, set and return it
        if (actualGoal.length == 1) {
            currentTargetCell = actualGoal[0];
            return currentTargetCell;
        }

        // If no target cell is locked yet, select the absolute closest
        if (currentTargetCell == null) {
            Pose2D bestCell = actualGoal[0];
            double minDistance = distanceTo(robotX, robotY, bestCell);

            for (int i = 1; i < actualGoal.length; i++) {
                double dist = distanceTo(robotX, robotY, actualGoal[i]);
                if (dist < minDistance) {
                    minDistance = dist;
                    bestCell = actualGoal[i];
                }
            }
            currentTargetCell = bestCell;
            return currentTargetCell;
        }

        // Check distance to currently locked target cell
        double currentDist = distanceTo(robotX, robotY, currentTargetCell);

        // Check if another candidate cell is closer by more than cellSwitchMargin
        for (Pose2D candidate : actualGoal) {
            if (candidate == currentTargetCell) continue;

            double candidateDist = distanceTo(robotX, robotY, candidate);

            // Only switch targets if candidate is closer by at least cellSwitchMargin (hysteresis threshold)
            if (candidateDist < currentDist - cellSwitchMargin) {
                currentTargetCell = candidate;
                currentDist = candidateDist;
            }
        }

        return currentTargetCell;
    }

    /**
     * Calculates distance in inches between robot position (X, Y) and a target Pose2D.
     *
     * @param robotX Robot X coordinate in inches
     * @param robotY Robot Y coordinate in inches
     * @param target Target Pose2D
     * @return Euclidean distance in inches
     */
    public double distanceTo(double robotX, double robotY, Pose2D target) {
        if (target == null) return Double.MAX_VALUE;
        double dx = robotX - target.getX(DistanceUnit.INCH);
        double dy = robotY - target.getY(DistanceUnit.INCH);
        distance = Math.hypot(dx, dy);
        return distance;
    }

    private void updateDistanceAndZone() {
        if (distance > closeEdge && distance <= close) zone = 1;
        else if (distance > close && distance <= far) zone = 2;
        else if (distance > far && distance <= farEdge) zone = 3;
    }

    public double calculatePower(double currentVelocity, double targetVelocity) {
        pidController.setPID(kP, kI, kD);
        updateDistanceAndZone();
        rpmTarget = Optional.ofNullable(targetRPM.get(zone)).orElse(0);
        if (rpmTarget <= 0) {
            pidController.reset();
            return 0.0;
        }

        //rpmMeasured
        double pidOutput = pidController.calculate(currentVelocity, targetVelocity);
        double ffOutput = kSShooter + kVShooter * (targetVelocity / 2000.0);
        return pidOutput + ffOutput;
    }
}
