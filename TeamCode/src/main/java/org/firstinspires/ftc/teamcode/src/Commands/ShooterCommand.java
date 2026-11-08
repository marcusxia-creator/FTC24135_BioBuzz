package org.firstinspires.ftc.teamcode.src.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;


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
        shooterSubsystem.closePollenGate();
    }

    @Override
    public void execute() {
        shooterSubsystem.shootRightFlywheel();
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
