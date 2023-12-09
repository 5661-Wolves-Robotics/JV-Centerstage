package org.firstinspires.ftc.teamcode.util.collision;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.util.collision.shapes.Box;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class CollisionDomain {

    List<Box> boxes;

    private CollisionDomain(List<Box> boxes){
        this.boxes = boxes;
    }

    public Vector2d checkBox(Box obj){
        Vector2d rect1_x = obj.pose.headingVec();
        Vector2d rect1_y = rect1_x.rotated(Math.toRadians(90));

        Vector2d corner1_x = rect1_x.times(obj.width / 2.0);
        Vector2d corner1_y = rect1_y.times(obj.height / 2.0);

        Vector2d center1 = obj.pose.vec();

        Vector2d[] corners1 = {
                corner1_x.plus(corner1_y).plus(center1),
                corner1_x.minus(corner1_y).plus(center1),
                corner1_y.minus(corner1_x).plus(center1),
                corner1_x.times(-1).minus(corner1_y).plus(center1),
        };


        for(Box box : boxes){
            Vector2d rect2_x = box.pose.headingVec();
            Vector2d rect2_y = rect2_x.rotated(Math.toRadians(90));

            Vector2d corner2_x = rect2_x.times(box.width / 2.0);
            Vector2d corner2_y = rect2_y.times(box.height / 2.0);

            Vector2d center2 = box.pose.vec();

            Vector2d[] corners2 = {
                    corner2_x.plus(corner2_y).plus(center2),
                    corner2_x.minus(corner2_y).plus(center2),
                    corner2_y.minus(corner2_x).plus(center2),
                    corner2_x.times(-1).minus(corner2_y).plus(center2),
            };

            double[] projected1_x = new double[4];
            double[] projected1_y = new double[4];
            double[] projected2_x = new double[4];
            double[] projected2_y = new double[4];

            for(int i = 0; i < 4; i++){
                projected1_x[i] = dist(center2, rect2_x, corners1[i]);
                projected1_y[i] = dist(center2, rect2_y, corners1[i]);
            }
            for(int i = 0; i < 4; i++){
                projected2_x[i] = dist(center1, rect1_x, corners2[i]);
                projected2_y[i] = dist(center1, rect1_y, corners2[i]);
            }

            double min1_x = getMin(projected1_x), max1_x = getMax(projected1_x), min1_y = getMin(projected1_y), max1_y = getMax(projected1_y);
            double min2_x = getMin(projected2_x), max2_x = getMax(projected2_x), min2_y = getMin(projected2_y), max2_y = getMax(projected2_y);

            if(isHit(min1_x, max1_x, box.width / 2) && isHit(min1_y, max1_y, box.height / 2) && isHit(min2_x, max2_x, obj.width / 2) && isHit(min2_y, max2_y, obj.height / 2)){
                Vector2d diff = obj.pose.vec().minus(box.pose.vec()).rotated(-box.pose.getHeading());
                double x = diff.getX() / box.width;
                double y = diff.getY() / box.height;
                if(Math.abs(x) > Math.abs(y)) return new Vector2d(x * box.width, 0).rotated(box.pose.getHeading());
                else return new Vector2d(0, y * box.height).rotated(box.pose.getHeading());
            }
        }

        return null;
    }

    private boolean isHit(double min, double max, double size){
        return (min < 0 && max > 0
                || Math.abs(min) < size
                || Math.abs(max) < size);
    }

    private double getMax(double[] list){
        double max = list[0];
        for(int i = 1; i < list.length; i++){
            if(list[i] > max) max = list[i];
        }
        return max;
    }

    private double getMin(double[] list){
        double min = list[0];
        for(int i = 1; i < list.length; i++){
            if(list[i] < min) min = list[i];
        }
        return min;
    }

    private static Vector2d projectOnto(Vector2d direction, Vector2d origin, Vector2d point){
        double dot = direction.getX() * (point.getX() - origin.getX()) + direction.getY() * (point.getY() - origin.getY());
        return direction.times(dot).plus(origin);
    }

    private static double dist(Vector2d origin, Vector2d line, Vector2d corner){
        Vector2d projected = projectOnto(line, origin, corner);
        Vector2d diff = projected.minus(origin);
        boolean sign = (diff.getX() * line.getX()) + (diff.getY() * line.getY()) > 0;
        return diff.norm() * (sign ? 1 : -1);
    }

    public static class Builder{

        List<Box> objects = new ArrayList<>();

        public Builder(){}

        public Builder addColliders(Box... shapes){
            objects.addAll(Arrays.asList(shapes));
            return this;
        }

        public CollisionDomain build(){
            return new CollisionDomain(objects);
        }

    }

}
