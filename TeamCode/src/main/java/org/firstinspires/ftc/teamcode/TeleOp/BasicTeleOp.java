package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.src.Commands.RobotDriveCommand;
import org.firstinspires.ftc.teamcode.src.Subsystems.ControlSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.RobotDriveSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

@Config
@TeleOp(name = "Basic TeleOp", group = "TeleOp")
public class BasicTeleOp extends CommandOpMode {
    private RobotDriveSubsystem robotDrive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TransferSubsystem transfer;
    private ControlSubsystem controls;
    private GamepadEx gamepad1Ex;
    private GamepadEx gamepad2Ex;
    private RobotHardware robot;

    public static double actualShooterRPM;


    @Override
    public void initialize() {
        robot = new RobotHardware(hardwareMap);
        robot.init();

        robotDrive = new RobotDriveSubsystem(robot);
        intake = new IntakeSubsystem(robot);
        transfer = new TransferSubsystem(robot);
        shooter = new ShooterSubsystem(robot);
        controls = new ControlSubsystem();

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        // Right stick X/Y moves and left stick X rotates; either driver can take over each axis.
        robotDrive.setDefaultCommand(new RobotDriveCommand(
                robotDrive,
                () -> Math.abs(gamepad1Ex.getRightY()) > 0.05
                        ? -gamepad1Ex.getRightY() : -gamepad2Ex.getRightY(),
                () -> Math.abs(gamepad1Ex.getRightX()) > 0.05
                        ? gamepad1Ex.getRightX() : gamepad2Ex.getRightX(),
                () -> Math.abs(gamepad1Ex.getLeftX()) > 0.05
                        ? gamepad1Ex.getLeftX() : gamepad2Ex.getLeftX(),
                () -> gamepad1Ex.getButton(GamepadKeys.Button.PS)
        ));

        controls.configureControls(gamepad1Ex, gamepad2Ex, intake, shooter, transfer);
    }

    @Override
    public void preRun() {
        intake.setInitialServoPositions();
    }

    @Override
    public void run() {
        super.run();
        double targetShooterRPM = shooter.getTargetRPM();
        double targetShooterTPS = ShooterSubsystem.rpmToTicksPerSecond(targetShooterRPM);
        double actualShooterTicksPerSecond = shooter.getShooterTicksPerSecond();
        actualShooterRPM = Math.abs(ShooterSubsystem.ticksPerSecondToRPM(actualShooterTicksPerSecond));
        telemetry.addData("Run time timer type shi", getRuntime());

        telemetry.addData("Intake Target Power", "%.1f", intake.getIntakeMotorPower());
        telemetry.addData("Intake Active Power", "%.2f", robot.intakeMotor.getPower());
        telemetry.addData("Intake velocity", intake.getSampledIntakeTicksPerSecond());
        telemetry.addData("Intake RPM", "%.1f", intake.getIntakeRPM());
        telemetry.addData("Intake Full", intake.isIntakeFull());
        telemetry.addLine("--- SHOOTER TELEMETRY ---");
        telemetry.addData("Target RPM", "%.1f", targetShooterRPM);
        telemetry.addData("Target TPS", "%.1f", targetShooterTPS);
        telemetry.addData("Shooter On", shooter.isShooterOn());
        telemetry.addData("Shooter Ready", shooter.isFlywheelReady());
        telemetry.addData("Transfer Running", transfer.isTransferRunning());
        telemetry.addData("Transfer At Speed", transfer.isTransferReady());
        telemetry.addData("Right Flywheel RPM", actualShooterRPM);
        telemetry.addData("Right Flywheel TPS", actualShooterTicksPerSecond);
        telemetry.addData("Shooter power", robot.rightShooterMotor.getPower());
        telemetry.addData("Shooter Encoder Position", shooter.getShooterEncoderPosition());
        telemetry.update();
    }
}
