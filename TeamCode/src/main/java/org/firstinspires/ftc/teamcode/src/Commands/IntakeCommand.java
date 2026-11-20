package org.firstinspires.ftc.teamcode.src.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.TransferSubsystem;

public class IntakeCommand extends CommandBase {
    private static final double MIN_SPEED_FRACTION = 0.20;
    private static final double SPIN_UP_SECONDS = 0.50;
    private static final double LOW_SPEED_SECONDS = 0.25;

    private final IntakeSubsystem intakeSubsystem;
    private final TransferSubsystem transferSubsystem;
    private final Runnable startRollers;
    private final ElapsedTime runTime = new ElapsedTime();
    private double lowSpeedSince;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem,
                         Runnable startRollers) {
        this.intakeSubsystem = intakeSubsystem;
        this.transferSubsystem = transferSubsystem;
        this.startRollers = startRollers;
        addRequirements(intakeSubsystem, transferSubsystem);
    }

    @Override
    public void initialize() {
        startRollers.run();
        transferSubsystem.setTransferSpeedRPM(100);
        transferSubsystem.runTransferAtTargetSpeed();
        lowSpeedSince = Double.NaN;
        runTime.reset();
    }

    @Override
    public boolean isFinished() {
        double elapsed = runTime.seconds();
        double targetRPM = intakeSubsystem.getCollectionTargetRPM();
        double measuredRPM = Math.abs(intakeSubsystem.getFeederCurrentRPM());
        if (elapsed < SPIN_UP_SECONDS || targetRPM <= 0.0 || !Double.isFinite(measuredRPM)
                || measuredRPM >= MIN_SPEED_FRACTION * targetRPM) {
            lowSpeedSince = Double.NaN;
            return false;
        }

        if (Double.isNaN(lowSpeedSince)) {
            lowSpeedSince = elapsed;
        }
        return elapsed - lowSpeedSince >= LOW_SPEED_SECONDS;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        transferSubsystem.stopTransfer();
    }
}
