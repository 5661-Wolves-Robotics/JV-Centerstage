package org.firstinspires.ftc.teamcode.util.collision.shapes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Box extends Point{

    public double width = 0;
    public double height = 0;

    public Box(Pose2d pose, double width, double height) {
        super(pose);
        this.width = width;
        this.height = height;
    }
}
