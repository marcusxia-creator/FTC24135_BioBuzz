package org.firstinspires.ftc.teamcode.src.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.src.Subsystems.RobotDriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class RobotDriveCommand extends CommandBase {
    private final RobotDriveSubsystem robotDrive;

    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;
    private final BooleanSupplier ps;

    public RobotDriveCommand(RobotDriveSubsystem robotDrive, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn, BooleanSupplier ps) {
        this.robotDrive = robotDrive;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        this.ps = ps;

        addRequirements(robotDrive);
    }
    @Override
    public void execute() {
        robotDrive.mecanumDrive(
                forward.getAsDouble(),
                strafe.getAsDouble(),
                turn.getAsDouble(),
                ps.getAsBoolean());
    }

}

