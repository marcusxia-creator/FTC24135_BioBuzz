package org.firstinspires.ftc.teamcode.src.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.TransferSubsystem;

public class ShooterCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final TransferSubsystem transferSubsystem;
    public ShooterCommand(ShooterSubsystem shooterSubsystem, TransferSubsystem transferSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.transferSubsystem = transferSubsystem;

        addRequirements(shooterSubsystem, transferSubsystem);
    }
}
