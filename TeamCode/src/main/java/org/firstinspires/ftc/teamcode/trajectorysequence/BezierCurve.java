package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class BezierCurve {

    private Vector2d p1, p2, p3, p4;

    public BezierCurve(Vector2d p1, Vector2d p2, Vector2d p3, Vector2d p4){
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
        this.p4 = p4;
    }

    public Vector2d evaluate(double t){
        double t_ = 1 - t;
        return p1.times(t_*t_*t_).
                plus(p2.times(3*t*t_*t_)).
                plus(p3.times(3*t*t*t_)).
                plus(p4.times(t*t*t));
    }

    public Vector2d derivative(double t){
        double t_ = 1 - t;
        double t2 = 2*t*t_;
        return p1.times(-3*t_*t_)
                .plus(p2.times(3 * ((t_*t_) - t2)))
                .plus(p3.times(3 * (t2 - (t*t))))
                .plus(p4.times(3 * t*t));
    }
}
