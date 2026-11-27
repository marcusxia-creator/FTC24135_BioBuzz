package org.firstinspires.ftc.teamcode.src.UniversalTools;

import com.acmerobotics.dashboard.config.Config;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.*;

@Config
public final class DistanceShooterRPM {
    public static double manualRPM = rightShooterTargetRPM;
    public static double nearDistanceInches = 0.0;
    public static double nearRPM = 0.0;
    public static double farDistanceInches = 0.0;
    public static double farRPM = 0.0;

    private final ShotTargeting targeting;
    private boolean automatic;
    private double lastAutomaticRPM = Double.NaN;
    private String status = "Idle";

    public DistanceShooterRPM(ShotTargeting targeting) {
        this.targeting = targeting;
    }

    public void toggleMode() {
        automatic = !automatic;
        clearShot();
    }

    public void clearShot() {
        lastAutomaticRPM = Double.NaN;
        status = "Idle";
    }

    public double selectRPM() {
        if (automatic) {
            if (targeting.update()) {
                double calibrated = interpolate(targeting.getDistance());
                if (validRPM(calibrated)) {
                    lastAutomaticRPM = calibrated;
                    status = "Using distance calibration";
                    return calibrated;
                }
                status = "Distance outside calibration or invalid settings";
            } else {
                status = targeting.getStatus();
            }
            if (validRPM(lastAutomaticRPM)) {
                status += " / holding last auto RPM";
                return lastAutomaticRPM;
            }
            status += " / using manual RPM";
        } else {
            status = "Using manual RPM";
        }
        return clampRPM(manualRPM);
    }

    static double interpolate(double distance) {
        if (!(nearDistanceInches >= 0.0 && farDistanceInches > nearDistanceInches)
                || !Double.isFinite(farDistanceInches)
                || !(distance >= nearDistanceInches && distance <= farDistanceInches)
                || !validRPM(nearRPM) || !validRPM(farRPM)) {
            return Double.NaN;
        }
        double fraction = (distance - nearDistanceInches) / (farDistanceInches - nearDistanceInches);
        return nearRPM + fraction * (farRPM - nearRPM);
    }

    public void adjustManualRPM(double change) {
        manualRPM = clampRPM(manualRPM + change);
    }

    private static boolean validRPM(double rpm) {
        return Double.isFinite(rpm) && rpm > 0.0
                && Double.isFinite(maxShooterRpm) && rpm <= maxShooterRpm;
    }

    private static double clampRPM(double rpm) {
        if (!Double.isFinite(rpm) || !Double.isFinite(maxShooterRpm) || maxShooterRpm <= 0.0) {
            return 0.0;
        }
        return Math.max(0.0, Math.min(maxShooterRpm, rpm));
    }

    public String getMode() { return automatic ? "Auto RPM" : "Manual Fixed RPM"; }
    public String getStatus() { return status; }

}
