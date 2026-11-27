package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.*;

@Config
public class TransferSubsystem extends SubsystemBase {
    private static final double TRANSFER_RPM_TOLERANCE = transferToleranceRPM;
    // Gear Ratio: Motor Gear Teeth / Flywheel Gear Teeth (e.g. 30T motor gear driving 40T flywheel gear)
    public static double GEAR_RATIO = 20.0 / 20.0;

    public static double FEEDER_TICK_PER_REV = 145.1; // 1150RPM gearbox 5.2:1; gear ratio 1:1 drive

    public static double transferKp = 0.5;
    public static double transferKi = 0.0;
    public static double transferKd = 0.0;
    public static double transferKf = 1.0;
    public static double transferStartupPower = 0.22;
    private static final double STOPPED_SPEED_RPM = 10.0;
    private static final double NOMINAL_MAX_RPM = 1150.0*0.9;

    private final RobotHardware robot;
    private final PIDFController transferPid =
            new PIDFController(transferKp, transferKi, transferKd, 0.0);
    private boolean running;

    private double transferSpeedTargetRPM = Double.NaN;
    private double transferSpeedRPM = Double.NaN;

    public TransferSubsystem(RobotHardware robot) {
        this.robot = robot;
    }
    @Override 
    public void periodic() {

        sampleTransferVelocity();
        if (running) {
            updateTransferPower();
        }
    }

    ///conversion 
    private static double ticksPerSecondToRPM(double ticksPerSecond, double tickPerRev, double gearRatio) {
        double motorRPM = (ticksPerSecond * 60.0) / tickPerRev;
        return motorRPM * gearRatio;
    }

    public static double rpmToTicksPerSecond(double RPM, double GEAR_RATIO, double TICKS_PER_REV) {
        double motorRPM = RPM / GEAR_RATIO;
        return (motorRPM * TICKS_PER_REV) / 60.0;
    }

    public void sampleTransferVelocity(){
        transferSpeedRPM = ticksPerSecondToRPM(robot.transferMotor.getVelocity(), FEEDER_TICK_PER_REV, GEAR_RATIO);
    }
    //setter
    public void setTransferSpeedRPM(double rpm) {
        if (Double.compare(transferSpeedTargetRPM, rpm) != 0) {
            transferPid.reset();
        }
        transferSpeedTargetRPM = rpm;
        if (!Double.isFinite(rpm) || rpm == 0.0) {
            stopTransfer();
        }
    }

    public void runTransferAtTargetSpeed() {
        if (Double.isFinite(transferSpeedTargetRPM) && transferSpeedTargetRPM != 0.0) {
            running = true;
        } else {
            stopTransfer();
        }
    }

    private void updateTransferPower() {
        if (!Double.isFinite(transferSpeedRPM)) {
            transferPid.reset();
            robot.transferMotor.setPower(0.0);
            return;
        }

        double normalizedTarget = transferSpeedTargetRPM / NOMINAL_MAX_RPM;
        double normalizedMeasured = transferSpeedRPM / NOMINAL_MAX_RPM;
        transferPid.setPIDF(transferKp, transferKi, transferKd, 0.0);
        double power = transferPid.calculate(normalizedMeasured, normalizedTarget)
                + transferKf * normalizedTarget;

        if (!Double.isFinite(power)) {
            transferPid.reset();
            robot.transferMotor.setPower(0.0);
            return;
        }
        double direction = Math.signum(normalizedTarget);
        double powerInTargetDirection = direction * power;
        // Give low RPM targets enough power to break static friction from a stop.
        if (Math.abs(transferSpeedRPM) < STOPPED_SPEED_RPM
                && Math.abs(transferSpeedRPM) < Math.abs(transferSpeedTargetRPM)
                && powerInTargetDirection > 0.0 && Double.isFinite(transferStartupPower)) {
            powerInTargetDirection = Math.max(powerInTargetDirection, transferStartupPower);
        }
        robot.transferMotor.setPower(direction * Range.clip(powerInTargetDirection, 0.0, 1.0));
    }

    public boolean stopTransfer() {
        running = false;
        transferPid.reset();
        robot.transferMotor.setPower(0.0);
        return true;
    }

    ///Getter
    public double getTransferSpeedRPM() {
       return transferSpeedRPM;
    }

    public double getTransferPower() {
        return robot.transferMotor.getPower();
    }

    public boolean isRunning() {
        return running;
    }

    public double getFeederTargetRPM() {
        return transferSpeedTargetRPM;
    }


    public boolean isTransferAtTargetSpeedRPM() {
        double currentRPM = getTransferSpeedRPM();
        return running && transferSpeedTargetRPM > 0.0
                && Double.isFinite(currentRPM)
                && currentRPM > 0.0
                && Math.abs(currentRPM - transferSpeedTargetRPM) <= TRANSFER_RPM_TOLERANCE;
    }


    public void increaseRPM() {
        transferTargetRPM = Math.min(NOMINAL_MAX_RPM,transferSpeedTargetRPM + 25.0);
        //setTransferSpeedRPM(transferSpeedTargetRPM);
        //runTransferAtTargetSpeed();
    }

    public void decreaseRPM() {
        transferTargetRPM = Math.max(0.0, transferSpeedTargetRPM - 25.0);
        //setTransferSpeedRPM(transferSpeedTargetRPM);
        //runTransferAtTargetSpeed();
    }
}
