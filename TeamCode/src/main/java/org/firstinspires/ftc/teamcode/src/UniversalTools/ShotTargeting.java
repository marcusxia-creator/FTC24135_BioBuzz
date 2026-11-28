package org.firstinspires.ftc.teamcode.src.UniversalTools;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.teamcode.src.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public final class ShotTargeting {
    public enum Alliance { RED, BLUE }
    public enum Cell { LEFT, RIGHT }

    public static Alliance alliance = Alliance.RED;
    public static boolean startingPoseConfigured = false;
    public static double startXInches = 0.0;
    public static double startYInches = 0.0;
    public static double startHeadingDegrees = 0.0;
    public static double aimPowerPerRadian = 0.8;
    public static double maxTurnPower = 0.4;
    public static double aimToleranceDegrees = 2.0;
    public static double limelightAlignedTxDegrees = 0.0;
    public static boolean positiveXIsAudienceSide = true;

    private final GoBildaPinpointDriver pinpoint;
    private final LimelightSubsystem limelight;
    private Alliance selectedAlliance;
    private Cell selectedCell = Cell.LEFT;
    private boolean fieldPoseReady;
    private boolean sampled;
    private boolean valid;
    private double targetX = -12.0;
    private double targetY;
    private double distance = Double.NaN;
    private double headingError = Double.NaN;
    private String aimSource = "None";
    private double aimErrorDegrees = Double.NaN;
    private String status = "Waiting for start";

    public ShotTargeting(GoBildaPinpointDriver pinpoint) {
        this(pinpoint, null);
    }

    public ShotTargeting(GoBildaPinpointDriver pinpoint, LimelightSubsystem limelight) {
        this.pinpoint = pinpoint;
        this.limelight = limelight;
    }

    public void initializeFieldPose() {
        selectedAlliance = alliance;
        fieldPoseReady = false;
        if (!startingPoseConfigured || selectedAlliance == null
                || !withinField(startXInches) || !withinField(startYInches)
                || !Double.isFinite(startHeadingDegrees)) {
            status = "Set alliance and field starting pose";
            return;
        }
        if (pinpoint == null) {
            status = "Pinpoint missing";
            return;
        }

        try {
            pinpoint.update();
            if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
                status = "Pinpoint not ready at start";
                return;
            }
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startXInches, startYInches,
                    AngleUnit.DEGREES, startHeadingDegrees));
            fieldPoseReady = true;
            targetY = selectedAlliance == Alliance.RED ? -12.75 : 12.75;
            status = "Ready";
        } catch (RuntimeException failure) {
            status = "Pinpoint starting pose failed";
        }
    }

    public void beginLoop() {
        sampled = false;
        valid = false;
        distance = Double.NaN;
        headingError = Double.NaN;
        aimSource = "None";
        aimErrorDegrees = Double.NaN;
        if (fieldPoseReady) {
            status = "Idle";
        }
    }

    /** Reads Pinpoint at most once per loop, when either feature needs a target. */
    public boolean update() {
        if (sampled || !fieldPoseReady) {
            return valid;
        }
        sampled = true;

        try {
            pinpoint.update();
            if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
                status = "Pinpoint not ready";
                return false;
            }

            double x = pinpoint.getPosX(DistanceUnit.INCH);
            double y = pinpoint.getPosY(DistanceUnit.INCH);
            double heading = pinpoint.getHeading(AngleUnit.RADIANS);
            if (!withinField(x) || !withinField(y) || !Double.isFinite(heading)) {
                status = "Pinpoint pose invalid";
                return false;
            }

            if (x != 0.0) {
                targetX = Math.copySign(13.0, x);
            }
            double dx = targetX - x;
            double dy = targetY - y;
            distance = Math.hypot(dx, dy);
            if (distance > 0.0) {
                double error = Math.atan2(dy, dx) - heading;
                headingError = Math.atan2(Math.sin(error), Math.cos(error));
            }
            valid = true;
            status = "Pinpoint Tracking";
        } catch (RuntimeException failure) {
            status = "Pinpoint read failed";
        }
        return valid;
    }

    public double getAimTurn() {
        if (!(aimPowerPerRadian > 0.0) || !Double.isFinite(aimPowerPerRadian)
                || !(maxTurnPower > 0.0 && maxTurnPower <= 1.0)
                || !(aimToleranceDegrees >= 0.0 && aimToleranceDegrees < 180.0)) {
            return Double.NaN;
        }

        update();
        // Drive power and Limelight tx are clockwise-positive. Pinpoint is counterclockwise-positive.
        aimErrorDegrees = -Math.toDegrees(headingError);
        aimSource = Double.isFinite(aimErrorDegrees) ? "Pinpoint" : "None";
        if (limelight != null && selectedAlliance != null) {
            boolean audienceSide = (targetX > 0.0) == positiveXIsAudienceSide;
            int firstTagId = selectedAlliance == Alliance.RED
                    ? (audienceSide ? 34 : 30)
                    : (audienceSide ? 38 : 42);
            double visionError = limelight.getAimTxDegrees(firstTagId) - limelightAlignedTxDegrees;
            if (Double.isFinite(visionError)) {
                aimErrorDegrees = visionError;
                aimSource = "Limelight";
            }
        }

        if (!Double.isFinite(aimErrorDegrees)) {
            return Double.NaN;
        }
        if (Math.abs(aimErrorDegrees) <= aimToleranceDegrees) {
            return 0.0;
        }
        double turn = Math.toRadians(aimErrorDegrees) * aimPowerPerRadian;
        return Math.max(-maxTurnPower, Math.min(maxTurnPower, turn));
    }

    private static boolean withinField(double position) {
        return Math.abs(position) <= 72.0;
    }

    public void toggleCell() {
        selectedCell = selectedCell == Cell.LEFT ? Cell.RIGHT : Cell.LEFT;
        targetX = selectedCell == Cell.LEFT ? -13.0 : 13.0;
        beginLoop();
    }

    public Alliance getAlliance() { return selectedAlliance; }
    public Cell getSelectedCell() { return selectedCell; }
    public double getTargetX() { return targetX; }
    public double getTargetY() { return targetY; }
    public double getDistance() { return distance; }
    public double getHeadingErrorDegrees() { return Math.toDegrees(headingError); }
    public double getAimErrorDegrees() { return aimErrorDegrees; }
    public String getAimSource() { return aimSource; }
    public String getStatus() { return status; }
}
