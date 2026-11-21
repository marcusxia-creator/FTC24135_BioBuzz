package org.firstinspires.ftc.teamcode.src.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.TransferSubsystem;

public class ShooterCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final TransferSubsystem transferSubsystem;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, TransferSubsystem transferSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.transferSubsystem = transferSubsystem;
        addRequirements(shooterSubsystem, transferSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.closePollenGate();
    }

    @Override
    public void execute() {
        // Regulate the flywheel while keeping the limiter engaged during initial spool-up.
        shooterSubsystem.runFlywheel();
        boolean flywheelReady = shooterSubsystem.isFlywheelReady();
        if (!flywheelReady) {
            transferSubsystem.stopTransfer();
            return;
        }
        transferSubsystem.runTransfer();
    }

    @Override
    public void end(boolean interrupted) {
        transferSubsystem.stopTransfer();
        shooterSubsystem.closePollenGate();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
