package org.firstinspires.ftc.teamcode.bot;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Robot<T extends Drive> {

    protected static Pose2d pose = new Pose2d(0, 0, 0);
    protected T drive = null;
    protected ElapsedTime time = new ElapsedTime();
    protected MultipleTelemetry telemetry = null;

    public Robot(){}

    public void init(HardwareMap hardwareMap, T drive, Pose2d pose, MultipleTelemetry telemetry) {
        this.drive = drive;

        drive.setPoseEstimate(pose);

        this.telemetry = telemetry;
    }

    public abstract void init(HardwareMap hardwareMap, Pose2d pose, MultipleTelemetry telemetry);

    public T getDrive(){
        return drive;
    }

    public void storePose(){
        pose = drive.getPoseEstimate();
    }

    public static Pose2d getStoredPose() {
        return pose;
    }

    public abstract void update();
}
