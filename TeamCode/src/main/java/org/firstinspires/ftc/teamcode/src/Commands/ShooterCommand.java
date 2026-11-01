package org.firstinspires.ftc.teamcode.src.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;

import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.intakeShootingPower;

public class ShooterCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final ElapsedTime timer = new ElapsedTime();

    public ShooterCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        shooterSubsystem.closePollenGate();
    }

    @Override
    public void execute() {
        // Runs intake feeder motor at 0.9 power
        shooterSubsystem.setFeederPower(intakeShootingPower);

        // After 0.75 seconds, automatically lift pollen gate while keeping intake running
        if (timer.seconds() >= 0.75) {
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
}
