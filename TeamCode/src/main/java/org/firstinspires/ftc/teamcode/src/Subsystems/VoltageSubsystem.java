package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.REF_VOLTAGE;

@Config
public class VoltageSubsystem extends SubsystemBase {
    public static double FILTER_TIME_CONSTANT_SECONDS = 0.15;

    private final VoltageSensor sensor;
    private double filteredVoltage = Double.NaN;
    private long lastValidSampleNanos;

    public VoltageSubsystem(VoltageSensor sensor) {
        this.sensor = sensor;
    }

    @Override
    public void periodic() {
        double voltage = sensor.getVoltage();
        if (!Double.isFinite(voltage) || voltage <= 0.0) {
            return;
        }

        long now = System.nanoTime();
        double timeConstant = FILTER_TIME_CONSTANT_SECONDS;
        if (!Double.isFinite(filteredVoltage) || !Double.isFinite(timeConstant) || timeConstant <= 0.0) {
            filteredVoltage = voltage;
        } else {
            double elapsedSeconds = (now - lastValidSampleNanos) * 1e-9;
            double alpha = -Math.expm1(-elapsedSeconds / timeConstant);
            filteredVoltage += alpha * (voltage - filteredVoltage);
        }
        lastValidSampleNanos = now;
    }

    public double getFilteredVoltage() {
        return filteredVoltage;
    }

    public double getCompensationFactor() {
        double factor = REF_VOLTAGE / filteredVoltage;
        return Double.isFinite(factor) && factor > 0.0 ? factor : 1.0;
    }
}
