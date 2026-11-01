package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.src.Commands.RobotDriveCommand;
import org.firstinspires.ftc.teamcode.src.Commands.ShooterCommand;
import org.firstinspires.ftc.teamcode.src.Subsystems.ControlSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.RobotDriveSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

@TeleOp(name = "Basic TeleOp", group = "TeleOp")
public class BasicTeleOp extends CommandOpMode {
    private RobotDriveSubsystem robotDrive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private ControlSubsystem controls;
    private GamepadEx gamepad1Ex;
    private GamepadEx gamepad2Ex;

    @Override
    public void initialize() {
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init();

        robotDrive = new RobotDriveSubsystem(robot);
        intake = new IntakeSubsystem(robot);
        shooter = new ShooterSubsystem(robot);
        controls = new ControlSubsystem();

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        // Right stick X/Y moves and left stick X rotates; either driver can take over each axis.
        robotDrive.setDefaultCommand(new RobotDriveCommand(
                robotDrive,
                () -> Math.abs(gamepad1Ex.getRightY()) > 0.05
                        ? -gamepad1Ex.getRightY() : -gamepad2Ex.getRightY(),
                () -> Math.abs(gamepad1Ex.getRightX()) > 0.05
                        ? gamepad1Ex.getRightX() : gamepad2Ex.getRightX(),
                () -> Math.abs(gamepad1Ex.getLeftX()) > 0.05
                        ? gamepad1Ex.getLeftX() : gamepad2Ex.getLeftX()
        ));

        controls.configureControls(gamepad1Ex, gamepad2Ex, intake, shooter);
    }

    @Override
    public void preRun() {
        intake.setInitialServoPositions();
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Intake Motor Power", intake.getIntakeMotorPower());
        telemetry.addLine("--- SHOOTER TELEMETRY ---");
        telemetry.addData("Target Speed", "%.0f RPM (%.0f TPS)", shooter.getTargetRPM(), shooter.getTargetTicksPerSecond());
        telemetry.addData("Actual Speed", "%.0f RPM (%.0f TPS)", shooter.getShooterRPM(), shooter.getShooterTicksPerSecond());
        telemetry.update();
    }
}
