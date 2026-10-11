package org.firstinspires.ftc.teamcode.TeleOp.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.TransferSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final TransferSubsystem transferSubsystem;
    public IntakeCommand(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.transferSubsystem = transferSubsystem;
        addRequirements(intakeSubsystem, transferSubsystem);
    }
}
