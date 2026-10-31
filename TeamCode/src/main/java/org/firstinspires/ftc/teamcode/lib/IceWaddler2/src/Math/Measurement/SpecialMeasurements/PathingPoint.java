package org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.SpecialMeasurements;

import static org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Units.*;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Vector;

/// A measurement used to store the position and velocity of the robot while following a path. The velocity is assumed to be perfectly tangental to the path
public class PathingPoint {
    Position position;
    Scalar velocity;


    public PathingPoint(Position position, Scalar velocity){
        this.position=position;
        if(!velocity.getDimensions().equals(Dimensions.velocity)){throw new DimMismatch(velocity.getDimensions(),"velocity");}
        this.velocity=velocity;
    }

    public PathingPoint(double x, double y, Unit unit, double degrees, Scalar velocity){
        this(new Position(new Vector(x,y,unit),new NormalizedAngle(degrees,deg)),velocity);
    }

    public PathingPoint(double x, double y, Unit unit, double degrees){
        this(x,y,unit,degrees,new Scalar(0,metersPerSecond));
    }

    public PathingPoint(Position position){
        this(position, new Scalar(0,metersPerSecond));
    }

    public Position getPosition() {
        return position;
    }

    public Scalar getVelocity(){
        return velocity;
    }
}
