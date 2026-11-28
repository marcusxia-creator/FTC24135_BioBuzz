package org.firstinspires.ftc.teamcode.src.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig;


public class ShooterCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final TransferSubsystem transferSubsystem;
    private final GamepadEx gamepad;

    public ShooterCommand(
            GamepadEx gamepad,
            ShooterSubsystem shooterSubsystem,
            IntakeSubsystem intakeSubsystem,
            TransferSubsystem transferSubsystem) {
        this.gamepad = gamepad;
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
        boolean flywheelValid = shooterSubsystem.runFlywheel(RobotActionConfig.shooterTargetVelocityRPM);
        if (!flywheelValid || !shooterSubsystem.isFlywheelReady()) {
            intakeSubsystem.stop();
            transferSubsystem.stopTransfer();
            return;
        }
        if (gamepad.wasJustPressed(GamepadKeys.Button.Y)) {
            intakeSubsystem.runShooterIntake();
            transferSubsystem.setTransferTargetRPM(RobotActionConfig.transferTargetRPM);
            transferSubsystem.runTransfer();
        }
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
