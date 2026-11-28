package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.src.Commands.RobotDriveCommand;
import org.firstinspires.ftc.teamcode.src.Subsystems.ControlSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.RobotDriveSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.VoltageSubsystem;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

@Config
@TeleOp(name = "BLUE 🐧🔵", group = "TeleOp")
public class Blue extends CommandOpMode {
    private RobotDriveSubsystem robotDrive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TransferSubsystem transfer;
    private ControlSubsystem controls;
    private GamepadEx gamepad1Ex;
    private GamepadEx gamepad2Ex;
    private RobotHardware robot;

    public static double actualShooterRPM;

    private final ElapsedTime driveTimer = new ElapsedTime();


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
        //shooter.closePollenGate();
        driveTimer.reset();
    }

    @Override
    public void run() {

        double dt = driveTimer.seconds();
        driveTimer.reset();

        super.run();
        double targetShooterRPM = shooter.getTargetRPM();
        double currentShooterRPM = shooter.getCurrentShooterRPM();
        double actualShooterTicksPerSecond = shooter.getShooterTicksPerSecond();
        actualShooterRPM = Math.abs(ShooterSubsystem.ticksPerSecondToRPM(actualShooterTicksPerSecond));
        telemetry.addData("Loop Frequency Hz", "%.1f", loopFrequencyHz(dt));
        telemetry.addData("Run time timer type shi", getRuntime());

        telemetry.addData("Intake Target Power", "%.1f", intake.getIntakeMotorPower());
        telemetry.addData("Intake Active Power", "%.2f", robot.intakeMotor.getPower());
        telemetry.addData("Intake velocity", intake.getSampledIntakeTicksPerSecond());
        telemetry.addData("Intake RPM", "%.1f", intake.getIntakeRPM());
        telemetry.addData("Intake Full", intake.isIntakeFull());
        telemetry.addLine("--- SHOOTER TELEMETRY ---");
        telemetry.addData("Shooter Target RPM", "%.1f", targetShooterRPM);
        telemetry.addData("Shooter Current RPM", "%.1f", currentShooterRPM);
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
    ///helper method to select the axis from either gamepad1 or gamepad2, prioritizing gamepad1 if it has a non-zero input.
    private static double selectAxis(double primary, double secondary) {
        return Math.abs(primary) > 0.05 ? primary : secondary;
    }

    private static double loopFrequencyHz(double dt) {
        return dt > 0.0 ? 1.0 / dt : 0.0;
    }
}
