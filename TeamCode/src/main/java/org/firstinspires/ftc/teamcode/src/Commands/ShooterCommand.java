package org.firstinspires.ftc.teamcode.src.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.TransferSubsystem;

public class ShooterCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final TransferSubsystem transferSubsystem;

    public ShooterCommand(
            ShooterSubsystem shooterSubsystem,
            IntakeSubsystem intakeSubsystem,
            TransferSubsystem transferSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.transferSubsystem = transferSubsystem;
        addRequirements(shooterSubsystem, intakeSubsystem, transferSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.closePollenGate();
        intakeSubsystem.stop();
        transferSubsystem.stopTransfer();
    }

    @Override
    public void execute() {
        // Regulate the flywheel while keeping the limiter engaged during initial spool-up.
        shooterSubsystem.runFlywheel();
        if (!shooterSubsystem.isFlywheelReady()) {
            intakeSubsystem.stop();
            transferSubsystem.stopTransfer();
            shooterSubsystem.closePollenGate();
            return;
        }
        intakeSubsystem.runShooterIntake();
        transferSubsystem.runTransfer();
        if (transferSubsystem.isTransferReady()) {
            shooterSubsystem.openPollenGate();
        }
        else shooterSubsystem.closePollenGate();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        transferSubsystem.stopTransfer();
        shooterSubsystem.closePollenGate();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
