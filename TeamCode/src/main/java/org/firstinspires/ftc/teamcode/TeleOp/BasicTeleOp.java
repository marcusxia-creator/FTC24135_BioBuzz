package org.firstinspires.ftc.teamcode.TeleOp;

import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;

import org.firstinspires.ftc.teamcode.TeleOp.Commands.RobotDriveCommand;
///Subsystems
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.RobotDriveSubsystem;

import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotHardware;

@TeleOp (name = "Basic TeleOp", group = "Test")

public class BasicTeleOp extends CommandOpMode {
    private RobotDriveSubsystem robotDrive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private GamepadEx gamepadDriver;
    private GamepadEx gamepadOperator;

    @Override
    public void initialize (){
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init();

        robotDrive = new RobotDriveSubsystem(robot);
        intake = new IntakeSubsystem(robot);
        shooter = new ShooterSubsystem(robot);
        gamepadDriver = new GamepadEx(gamepad1);
        gamepadOperator = new GamepadEx(gamepad2);

        robotDrive.setDefaultCommand(new RobotDriveCommand(
                robotDrive,
                () -> -gamepadDriver.getRightY(),
                () -> gamepadDriver.getRightX(),
                () -> gamepadDriver.getLeftX()
        ));
        ///------------------Gamepad-----------------------
        ///Run intake
        new GamepadButton(gamepadOperator, GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(new StartEndCommand(intake:: runRollers,intake::stop, intake));
        new GamepadButton (gamepadOperator, GamepadKeys.Button.DPAD_UP)
                .whenPressed (new InstantCommand(() -> intake.increasePower(), intake));
        new GamepadButton (gamepadOperator, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(()-> intake.decreasePower(), intake));
        ///RUN SHOOTER
        new GamepadButton(gamepadOperator, GamepadKeys.Button.X)
                .toggleWhenPressed(new StartEndCommand(shooter::runFlywheel,shooter::stop));
        new GamepadButton(gamepadOperator, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand (shooter :: increasePower, shooter));
        new GamepadButton (gamepadOperator, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand (shooter :: decreasePower, shooter));
        ///STOP
        new GamepadButton(gamepadOperator, GamepadKeys.Button.B)
                .or(new GamepadButton(gamepadDriver, GamepadKeys.Button.B))
                .whenActive(new InstantCommand(() -> {shooter.stop();
                    intake.stop();
                }, shooter, intake));
    }
    @Override
    public void run (){
        super.run();
        telemetry.addData("Intake Motor Power", intake.getIntakeMotorPower());
        telemetry.addData("Shooter Power",shooter.getshooterPower());
        telemetry.update();
    }
}