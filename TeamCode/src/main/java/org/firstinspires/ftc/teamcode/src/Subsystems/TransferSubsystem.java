package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.*;

public class TransferSubsystem extends SubsystemBase {
    private final RobotHardware robot;
    private double transferTargetRPM = transferMaxRPM;
    private boolean transferRunning;
    private final PIDFController transferPIDF =
            new PIDFController(transferKp, transferKi, transferKd, transferKf);

    public TransferSubsystem(RobotHardware robot) {
        this.robot = robot;
    }

    public void runTransfer() {
        transferRunning = true;
        if (transferTargetRPM <= 0.0) {
            stopTransfer();
            return;
        }

        double targetNorm = transferTargetRPM / transferMaxRPM;
        double currentNorm = getTransferRPM() / transferMaxRPM;
        transferPIDF.setPIDF(transferKp, transferKi, transferKd, transferKf);
        double output = transferPIDF.calculate(currentNorm, targetNorm);
        robot.transferMotor.setPower(Range.clip(output, 0.0, 1.0));
    }

    public void runTransfer(double targetRPM) {
        setTransferTargetRPM(targetRPM);
        runTransfer();
    }

    public void intakeTransfer (){
        robot.transferMotor.setPower(0.5);
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
        double motorRPM = robot.transferMotor.getVelocity() * 60.0 / transferTicksPerRev;
        return Math.abs(motorRPM * transferGearRatio);
    }

    public boolean isTransferRunning() {
        return transferRunning;
    }
}
