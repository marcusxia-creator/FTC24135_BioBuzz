package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.TeleOp.Commands.GamepadControl;
import org.firstinspires.ftc.teamcode.TeleOp.Commands.RobotDriveCommand;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.RobotDriveSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotHardware;

@TeleOp (name = "Basic TeleOp", group = "Test")
public class BasicTeleOp extends CommandOpMode {
    private RobotDriveSubsystem robotDrive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TransferSubsystem transfer;
    private GamepadEx gamepad;
    private GamepadControl gamepadControl;

    @Override
    public void initialize (){
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init();

        robotDrive = new RobotDriveSubsystem(robot);
        intake = new IntakeSubsystem(robot);
        shooter = new ShooterSubsystem(robot);
        transfer = new TransferSubsystem(robot);
        gamepad = new GamepadEx(gamepad1);

        // Initialize GamepadControl with subsystems; all button bindings live there now!
        gamepadControl = new GamepadControl(gamepad1, gamepad2, intake, shooter, transfer);
        CommandScheduler.getInstance().schedule(gamepadControl.new Update());

        robotDrive.setDefaultCommand(new RobotDriveCommand(
                robotDrive,
                () -> -gamepad.getRightY(),
                () -> gamepad.getRightX(),
                () -> gamepad.getLeftX()
        ));
    }

    @Override
    public void run (){
        super.run();
        telemetry.addData("Intake Motor Power", intake.getIntakeMotorPower());
        telemetry.addData("Shooter Power", shooter.getshooterPower());
        telemetry.addData("Target Velocity", shooter.getTargetVelocity());
        telemetry.addData("Current Velocity", shooter.getCurrentVelocity());
        telemetry.update();
    }
}
