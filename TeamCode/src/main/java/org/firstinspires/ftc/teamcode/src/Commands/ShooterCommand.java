package org.firstinspires.ftc.teamcode.src.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;

import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.intakeShootingPower;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.shooterLimiterRetractTimeSeconds;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.shooterSpoolUpTimeSeconds;

public class ShooterCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElapsedTime spoolUpTimer = new ElapsedTime();

    public ShooterCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        spoolUpTimer.reset();
        shooterSubsystem.closePollenGate();
    }

    @Override
    public void execute() {
        // Regulate the flywheel while keeping the limiter engaged during initial spool-up.
        shooterSubsystem.runFlywheel();
        if (spoolUpTimer.seconds() >= shooterLimiterRetractTimeSeconds) {
            shooterSubsystem.openPollenGate();
        } else {
            shooterSubsystem.closePollenGate();
        }

        // Reference-speed feeder control is disabled
        // boolean referenceReady = shooterSubsystem.updateShooterSpeedReference();
        boolean spoolUpComplete = spoolUpTimer.seconds() >= shooterSpoolUpTimeSeconds;
        if (spoolUpComplete) {
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
