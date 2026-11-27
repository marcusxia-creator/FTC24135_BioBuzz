package org.firstinspires.ftc.teamcode.src.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig;

public class IntakeCommand extends CommandBase {
    private static final double TRANSFER_INTAKE_POWER_CUTOFF = 0.93;
    private static final double MIN_SPEED_FRACTION = 0.9;
    private static final double SPIN_UP_SECONDS = 0.50;
    private static final double LOW_SPEED_SECONDS = 0.25;

    private final IntakeSubsystem intakeSubsystem;
    private final TransferSubsystem transferSubsystem;
    private final Runnable startRollers;
    private final ElapsedTime runTime = new ElapsedTime();
    private double lowSpeedSince;
    private boolean intakeReachedTransferPowerCutoff;

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
        intakeReachedTransferPowerCutoff = intakeSubsystem.getIntakeMotorPower() >= TRANSFER_INTAKE_POWER_CUTOFF;
        lowSpeedSince = Double.NaN;
        runTime.reset();
    }
    
    @Override 
    public void execute() {
        double intakePower = intakeSubsystem.getIntakeMotorPower();
        // The default intake power is below the cutoff. Apply it after a downward crossing.
        if (intakePower >= TRANSFER_INTAKE_POWER_CUTOFF) {
            intakeReachedTransferPowerCutoff = true;
        } else if (intakeReachedTransferPowerCutoff) {
            transferSubsystem.stopTransfer();
            return;
        }

        transferSubsystem.setTransferSpeedRPM(RobotActionConfig.IntakeTransferTargetRPM);
        transferSubsystem.runTransferAtTargetSpeed();
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
