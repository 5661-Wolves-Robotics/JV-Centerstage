package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.collision.CollisionDomain;
import org.firstinspires.ftc.teamcode.util.collision.shapes.Box;

public class FieldConstants {

    public static Pose2d RED_BOARD = new Pose2d(60, -36, 0);
    public static Pose2d BLUE_BOARD = new Pose2d(60, 36, 0);

    public static Pose2d BLUE_BACKSTAGE_START = new Pose2d(12, 64, Math.toRadians(90));
    public static Pose2d BLUE_FRONTSTAGE_START = new Pose2d(-36, 64, Math.toRadians(90));
    public static Pose2d RED_BACKSTAGE_START = new Pose2d(12, -64, Math.toRadians(270));
    public static Pose2d RED_FRONTSTAGE_START = new Pose2d(-36, -64, Math.toRadians(270));

    public static CollisionDomain collision = new CollisionDomain.Builder()
            .addColliders(new Box(RED_BOARD, 10, 28))
            .build();

}
