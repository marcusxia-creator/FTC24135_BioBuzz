package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.lib.CommandBase.Action;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.*;

public class TransferSubsystem extends SubsystemBase {
    private final RobotHardware robot;
    private double transferTargetRPM = Double.NaN;
    private boolean transferRunning;
    private double sampledTransferTicksPerSecond;
    private final VoltageSubsystem voltageSubsystem;
    private final PIDFController transferPIDF =
            new PIDFController(transferKp, transferKi, transferKd, transferKf);

    private final double TICK_PER_REV = 145.1;
    public TransferSubsystem(RobotHardware robot) {
        this.robot = robot;
        this.voltageSubsystem = robot.voltageSubsystem;
    }

    @Override
    public void periodic() {
        sampledTransferTicksPerSecond = robot.transferMotor.getVelocity();

    }

    public boolean runTransfer() {
        if (!Double.isFinite(transferTargetRPM) || transferTargetRPM <= 0.0) {
            transferPIDF.reset();
            robot.transferMotor.setPower(0.0);
            transferRunning = false;
            return false;
        }

        transferRunning = true;
        double targetNorm = transferTargetRPM / transferMaxRPM;
        double currentNorm = getTransferRPM() / transferMaxRPM;
        updatePids();
        double output = transferPIDF.calculate(currentNorm, targetNorm);
        if (!Double.isFinite(output)) {
            transferPIDF.reset();
            robot.transferMotor.setPower(0.0);
            transferRunning = false;
            return false;
        }

        robot.transferMotor.setPower(Range.clip(output, 0.0, 1.0));
        return true;
    }

    public boolean runTransferWithVoltageCompensation() {
        if (!Double.isFinite(transferTargetRPM) || transferTargetRPM <= 0.0) {
            transferPIDF.reset();
            robot.transferMotor.setPower(0.0);
            transferRunning = false;
            return false;
        }

        transferRunning = true;
        double targetNorm = transferTargetRPM / transferMaxRPM;
        double currentNorm = getTransferRPM() / transferMaxRPM;
        updatePids();
        double output = transferPIDF.calculate(currentNorm, targetNorm);
        double power = output * voltageSubsystem.getCompensationFactor();
        if (!Double.isFinite(power)) {
            transferPIDF.reset();
            robot.transferMotor.setPower(0.0);
            transferRunning = false;
            return false;
        }

        robot.transferMotor.setPower(Range.clip(power, 0.0, 1.0));
        return true;
    }

    public void intakeTransfer() {
        transferRunning = true;
        double targetVelocityTickPerSecond = (transferTargetRPM * TICK_PER_REV) / 60.0;
        robot.transferMotor.setVelocity(targetVelocityTickPerSecond);
    }

    public void stopTransfer() {
        transferRunning = false;
        transferPIDF.reset();
        robot.transferMotor.setPower(0.0);
    }

    public double setTransferTargetRPM(double targetRPM) {
        transferTargetRPM = Range.clip(targetRPM, 0.0, transferMaxRPM);
        return transferTargetRPM;
    }

    public double getTransferTargetRPM() {
        return transferTargetRPM;
    }

    public double getTransferRPM() {
        double motorRPM = sampledTransferTicksPerSecond * 60.0 / transferTicksPerRev;
        return Math.abs(motorRPM * transferGearRatio);
    }

    public double getSampledTransferTicksPerSecond() {
        return sampledTransferTicksPerSecond;
    }

    public boolean isTransferRunning() {
        return transferRunning;
    }

    public boolean isTransferReady() {
        double targetTransferRPM = transferTargetRPM;
        return transferRunning
                && targetTransferRPM > 0.0
                && getTransferRPM() >= targetTransferRPM * shooterReadyMinimumTargetFraction;
    }

    private void updatePids() {
        if (transferPIDF.getP() != transferKp) transferPIDF.setP(transferKp);
        if (transferPIDF.getI() != transferKi) transferPIDF.setI(transferKi);
        if (transferPIDF.getD() != transferKd) transferPIDF.setD(transferKd);
        if (transferPIDF.getF() != transferKf) transferPIDF.setF(transferKf);
    }

    public class ShootTransferAction implements Action {
        private boolean transferValid = true;

        @Override
        public void loop() {
            transferValid = runTransfer();
        }

        @Override
        public void shutdown() {
            stopTransfer();
        }

        @Override
        public boolean finished() {
            return !transferValid;
        }
    }
}
