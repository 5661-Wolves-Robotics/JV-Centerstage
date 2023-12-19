package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.collision.CollisionDomain;
import org.firstinspires.ftc.teamcode.util.collision.shapes.Box;

public class FieldConstants {

    public static Pose2d RED_BOARD = new Pose2d(64, -36, 0);
    public static Pose2d BLUE_BOARD = new Pose2d(64, 36, 0);

    public static Pose2d BLUE_BACKSTAGE_START = new Pose2d(12, 63, Math.toRadians(90));
    public static Pose2d BLUE_FRONTSTAGE_START = new Pose2d(-36, 63, Math.toRadians(90));
    public static Pose2d RED_BACKSTAGE_START = new Pose2d(12, -63, Math.toRadians(270));
    public static Pose2d RED_FRONTSTAGE_START = new Pose2d(-36, -63, Math.toRadians(270));

    public static CollisionDomain collision = new CollisionDomain.Builder()
            .addColliders(new Box(RED_BOARD, 16, 24))
            .addColliders(new Box(BLUE_BOARD, 16, 24))
            .build();

    public enum Side {
        BLUE,
        RED
    }

    public enum Stage{
        FRONT,
        BACK
    }

    public static Pose2d getFieldStartPose(Side side, Stage stage){
        return new Pose2d(stage == Stage.BACK ? 12 : -24, side == Side.RED ? -63 : 63, side == Side.RED ? Math.toRadians(90) : Math.toRadians(270));
    }

    @FunctionalInterface
    public interface EnumSupplier<T>{
        T getVal();
    }

}
