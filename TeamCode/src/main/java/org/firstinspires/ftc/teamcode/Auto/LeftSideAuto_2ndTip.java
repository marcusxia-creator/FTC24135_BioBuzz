package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.lib.CommandBase.PrebuiltActions.ActionParallel.TERMINATIONTYPE.ALL;
import static org.firstinspires.ftc.teamcode.lib.CommandBase.PrebuiltActions.ActionParallel.TERMINATIONTYPE.FIRST;
import static org.firstinspires.ftc.teamcode.lib.CommandBase.PrebuiltActions.ActionParallel.TERMINATIONTYPE.NONE;
import static org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Units.Unit.deg;
import static org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Units.Unit.feet;
import static org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Units.Unit.metersPerSecond;
import static org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Units.Unit.s;
import static org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Pathing.ArcDirection.COUNTERCLOCKWISE;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.flowerRampUpPos;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.gateUpPos;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.pollenDownPos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lib.CommandBase.Action;
import org.firstinspires.ftc.teamcode.lib.CommandBase.PrebuiltActions.ActionParallel;
import org.firstinspires.ftc.teamcode.lib.CommandBase.PrebuiltActions.ActionSeries;
import org.firstinspires.ftc.teamcode.lib.CommandBase.PrebuiltActions.Delay;
import org.firstinspires.ftc.teamcode.lib.CommandBase.ScheduledOpMode;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.IceWaddler;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.SpecialMeasurements.PathingPoint;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Position;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.SpecialMeasurements.PositiveAngle;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Vector;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Pathing.ArcDirection;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Pathing.PrebuiltMotionProfiles.SCurveHP;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Pathing.PrebuiltMotionProfiles.linearHP;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Pathing.PrebuiltMotionProfiles.maxSpeedMP;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Pathing.PrebuiltPathingElements.FixedArc;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Pathing.PrebuiltPathingElements.Line;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Pathing.PrebuiltPathingElements.holdPos;
import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

@Config
@Autonomous(name="2 Tip Left Side Auto (Only for second tip)",preselectTeleOp="Basic TeleOp")
public class LeftSideAuto_2ndTip extends ScheduledOpMode {
    RobotHardware robot;
    IceWaddler waddler;

    Action path;

    IntakeSubsystem intake;
    ShooterSubsystem shooter;

    //Pathing Positions. All positions are in x,y, as on a graph, and headings are clockwise from vertical
    public static Position startPoint=new Position(new Vector(-5.15,-1.40,feet),new NormalizedAngle(90,deg));//Position at which the robot starts. I think this position's actually illegal rn
    public static Position rightHive=new Position(new Vector(-4.6,-1,feet),new NormalizedAngle(90,deg));  //The position that the robot goes to to shoot into the right hive. We will only be shooting in this hive
    public static Position prep_flower2=new Position(new Vector(-3,-2,feet),new NormalizedAngle(-90,deg));
    public static Position flower2=new Position(new Vector(-4.6,-2,feet),new NormalizedAngle(-90,deg)); //The position that the robot goes to intake from the second flower. This is the flower closest to the right hive.
    public static Position post_flower2=new Position(new Vector(-3,-2,feet),new NormalizedAngle(-90,deg));
    public static Position parkPoint=new Position(new Vector(-3.5,-4.6,feet),new NormalizedAngle(-90,deg));//This is the position at which the robot parks at the end of auto

    //Timings
    public static Scalar waitTime=new Scalar(9,s);
    public static Scalar spoolTime=new Scalar(2,s);
    public static Scalar shootTime=new Scalar(2.5,s);//Amount of time needed to consistently shoot all four pollen
    public static Scalar intakeTime=new Scalar(2,s);//Amount of time to consistently intake all four pollen from flower

    public static Scalar wallIntakeSpeed=new Scalar(1,metersPerSecond);

    @Override
    public void init() {
        robot=new RobotHardware(hardwareMap);
        robot.init();

        waddler=new IceWaddler(robot.driveTrain, robot.localizer);
        waddler.init(startPoint,false);

        intake=new IntakeSubsystem(robot);
        shooter=new ShooterSubsystem(robot);

        //All just inits, like in teleop

        //softstart ramp servos
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        robot.pollenLimitingServo.setPosition(pollenDownPos);
        robot.gateServo.setPosition(gateUpPos);

        //The path is an action series, which starts at action 1 and moves to the next as soon as the current action is complete

        path=new ActionSeries(
                waddler.new InitPath(new PathingPoint(startPoint)),//This sets the first point in the path

                new ActionParallel(FIRST,new Delay(waitTime),
                        waddler.new MotionAction(new holdPos(new String[]{})),
                        new ActionSeries(new Delay(waitTime.sub(spoolTime)),new ActionParallel(NONE,shooter.new spoolFlywheelAction()))),

                //move and shoot in right hive
                //This actionParallel moves toward the right hive shooting position while simultaneously spooling up the flywheel. The `FIRST` termination type means that this ActionParellel ends and moves to the next action when the robot reaches the end point and the `motionaction` ends
                new ActionParallel(FIRST,
                        waddler.new MotionAction(new Line(new PathingPoint(rightHive), new maxSpeedMP(), new linearHP(), new String[]{})),//Drives in a straight line towards the shooting position. This is the terminating action
                        shooter.new spoolFlywheelAction()),//This action spools up the flywheel without shooting
                //This actionParallel simultaneously runs the shooter while holding the robot still for the defined shootTime. The FIRST termination type means that this action ends when the shooting delay ends
                new ActionParallel(FIRST,
                        new Delay(shootTime),shooter.new ShootAction(),
                        waddler.new MotionAction(new holdPos(new String[]{}))),//Holds the robot still

                //Move and intake from the second flower
                //As before, this action moves toward the flower in a straight line while running the intake to lower the ramp
                waddler.new MotionAction(new Line(new PathingPoint(prep_flower2), new maxSpeedMP(), new linearHP(), new String[]{})),
                new ActionParallel(ALL,
                        intake.new flowerIntakeAction(),
                        waddler.new MotionAction(new Line(new PathingPoint(flower2), new maxSpeedMP(), new linearHP(), new String[]{}))),
                new ActionParallel(FIRST,
                        new Delay(intakeTime),intake.new flowerIntakeAction(),
                        waddler.new MotionAction(new holdPos(new String[]{}))),
                new ActionParallel(ALL,
                        intake.new flowerIntakeAction(),
                        shooter.new spoolFlywheelAction(),
                        waddler.new MotionAction(new Line(new PathingPoint(post_flower2), new maxSpeedMP(), new linearHP(), new String[]{}))),

                //Move to the right hive shooting pos and shoot
                new ActionParallel(ALL,
                        shooter.new spoolFlywheelAction(),
                        waddler.new MotionAction(new Line(new PathingPoint(rightHive), new maxSpeedMP(), new linearHP(), new String[]{})),
                        intake.new flowerIntakeAction()),
                new ActionParallel(FIRST,
                        new Delay(shootTime),shooter.new ShootAction(),
                        waddler.new MotionAction(new holdPos(new String[]{}))),

                //Move to park
                waddler.new MotionAction(new Line(new PathingPoint(parkPoint), new maxSpeedMP(), new linearHP(), new String[]{}))
        );

        //Register the path into the root action
        rootAction=new ActionParallel(NONE,
                path//,telemetry or smth, if needed
        );
    }
}
