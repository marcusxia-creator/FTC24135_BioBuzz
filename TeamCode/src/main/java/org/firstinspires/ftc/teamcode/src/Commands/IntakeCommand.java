package org.firstinspires.ftc.teamcode.src.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import java.util.function.Supplier;

import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig;

public class IntakeCommand extends CommandBase {
    public enum IntakeMode {
        POLLEN,
        NECTAR,
        FLOWER
    }

    private final IntakeSubsystem intakeSubsystem;
    private final TransferSubsystem transferSubsystem;
    private final Supplier<IntakeMode> modeSupplier;

    private ElapsedTime runTime = new ElapsedTime();
    private double SPIN_UP_SECONDS = 0.5;
    private double LOW_SPEED_SECONDS = 0.25;
    private double MIN_SPEED_FRACTION = 0.9;

    private double lowSpeedSince;

    public IntakeCommand(
            IntakeSubsystem intakeSubsystem,
            TransferSubsystem transferSubsystem,
            Supplier<IntakeMode> modeSupplier) {
        this.intakeSubsystem = intakeSubsystem;
        this.transferSubsystem = transferSubsystem;
        this.modeSupplier = modeSupplier;
        addRequirements(intakeSubsystem, transferSubsystem);
    }

    @Override
    public void initialize() {
        switch (modeSupplier.get()) {
            case NECTAR:
                intakeSubsystem.runRollersNectar();
                break;
            case FLOWER:
                intakeSubsystem.runRollersFlower();
                break;
            case POLLEN:
            default:
                intakeSubsystem.runRollersPollen();
                break;
        }
        transferSubsystem.intakeTransfer();
    }

    @Override
    public void execute() {
        transferSubsystem.setTransferTargetRPM(RobotActionConfig.intakeTransferRPM);
        transferSubsystem.runTransfer();
    }

    @Override
    public boolean isFinished() {
        double elapsed = runTime.seconds();
        double measuredRPM = Math.abs(intakeSubsystem.getIntakeRPM());
        if (elapsed < SPIN_UP_SECONDS || !Double.isFinite(measuredRPM)
                || measuredRPM >= MIN_SPEED_FRACTION * RobotActionConfig.intakeTargetRPM) {
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
