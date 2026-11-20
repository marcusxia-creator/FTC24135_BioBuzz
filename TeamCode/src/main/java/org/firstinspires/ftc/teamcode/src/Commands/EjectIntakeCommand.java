package org.firstinspires.ftc.teamcode.src.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;

public class EjectIntakeCommand extends CommandBase {
    private static final double STOP_DELAY_SECONDS = 0.15;
    private static final double REVERSE_RAMP_SECONDS = 0.30;
    private static final double MAX_REVERSE_POWER = 1.0;

    private final IntakeSubsystem intake;
    private long startNanos;

    public EjectIntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.prepareEject();
        startNanos = System.nanoTime();
    }

    @Override
    public void execute() {
        double elapsedSeconds = (System.nanoTime() - startNanos) / 1e9;
        double rampSeconds = elapsedSeconds - STOP_DELAY_SECONDS;
        if (rampSeconds <= 0.0) {
            return;
        }

        double fraction = Math.min(1.0, rampSeconds / REVERSE_RAMP_SECONDS);
        intake.setFeederPower(-MAX_REVERSE_POWER * fraction);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setFeederPower(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}