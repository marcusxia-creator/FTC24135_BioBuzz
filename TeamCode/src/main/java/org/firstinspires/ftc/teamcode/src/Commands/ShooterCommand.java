package org.firstinspires.ftc.teamcode.src.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Robot;

import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig;

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
        shooterSubsystem.openPollenGate();
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
            return;
        }
        intakeSubsystem.runShooterIntake();
        transferSubsystem.setTransferTargetRPM(RobotActionConfig.transferTargetRPM);
        transferSubsystem.runTransfer();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        transferSubsystem.stopTransfer();
        shooterSubsystem.closePollenGate();
        shooterSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
