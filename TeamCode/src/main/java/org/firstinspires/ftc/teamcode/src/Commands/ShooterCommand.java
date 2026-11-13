package org.firstinspires.ftc.teamcode.src.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;

import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.intakeShootingPower;

public class ShooterCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.resetShooterSpeedReference();
        shooterSubsystem.closePollenGate();
    }

    @Override
    public void execute() {
        // Regulate the flywheel and open the pollen gate while it spools up.
        shooterSubsystem.runFlywheel();
        shooterSubsystem.openPollenGate();

        // Recheck every scheduler cycle, stop feeding below target and resume when ready again.
        boolean shooterReady = shooterSubsystem.updateShooterSpeedReference();
        if (shooterReady) {
            intakeSubsystem.setFeederPower(intakeShootingPower);
        } else {
            intakeSubsystem.setFeederPower(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setFeederPower(0.0);
        shooterSubsystem.closePollenGate();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
