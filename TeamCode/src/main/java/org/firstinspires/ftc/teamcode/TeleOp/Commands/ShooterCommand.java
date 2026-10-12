package org.firstinspires.ftc.teamcode.TeleOp.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.TransferSubsystem;

public class ShooterCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final TransferSubsystem transferSubsystem;
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private static final double SPIN_UP_DELAY_SECONDS = 2.0;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, TransferSubsystem transferSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.transferSubsystem = transferSubsystem;
        addRequirements(shooterSubsystem, transferSubsystem);
    }

    @Override
    public void initialize() {
        // Start spinning up the shooter immediately
        shooterSubsystem.runFlywheel();
        // Keep transfer stopped during the spin-up window
        transferSubsystem.stop();
        // Reset timer
        spinUpTimer.reset();
    }

    @Override
    public void execute() {
        // Keep running the shooter flywheel software PID every loop
        shooterSubsystem.runFlywheel();

        // After 2 seconds have passed, start sweeping the transfer servo
        if (spinUpTimer.seconds() >= SPIN_UP_DELAY_SECONDS) {
            transferSubsystem.sweep();
        } else {
            transferSubsystem.stop();
        }
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
