package org.firstinspires.ftc.teamcode.src.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import java.util.function.Supplier;

import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.TransferSubsystem;

public class IntakeCommand extends CommandBase {
    public enum IntakeMode {
        POLLEN,
        NECTAR,
        FLOWER
    }

    private final IntakeSubsystem intakeSubsystem;
    private final TransferSubsystem transferSubsystem;
    private final Supplier<IntakeMode> modeSupplier;

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
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        transferSubsystem.stopTransfer();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.isIntakeFull();
    }
}
