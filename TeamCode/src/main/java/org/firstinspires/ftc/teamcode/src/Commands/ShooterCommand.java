package org.firstinspires.ftc.teamcode.src.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;

import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.intakeShootingPower;

public class ShooterCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElapsedTime timer = new ElapsedTime();

    public ShooterCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        shooterSubsystem.closePollenGate();
    }

    @Override
    public void execute() {
        // Keep the flywheel regulated while feeding balls into it.
        shooterSubsystem.runFlywheel();
        shooterSubsystem.setFeederPower(0.8);

        // After 0.75 seconds, automatically lift pollen gate while keeping intake running
        if (timer.seconds() > 0.3) {
            shooterSubsystem.openPollenGate();
        } else {
            shooterSubsystem.closePollenGate();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setFeederPower(0);
        shooterSubsystem.closePollenGate();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
