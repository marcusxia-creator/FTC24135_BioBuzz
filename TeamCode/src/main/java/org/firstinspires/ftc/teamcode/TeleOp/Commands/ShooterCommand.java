package org.firstinspires.ftc.teamcode.TeleOp.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.TransferSubsystem;

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
        transferSubsystem.sweep();
    }

    @Override
    public void execute() {
        // Runs custom software PID calculation every loop cycle while held
        shooterSubsystem.runFlywheel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
        transferSubsystem.stop();
    }
}
