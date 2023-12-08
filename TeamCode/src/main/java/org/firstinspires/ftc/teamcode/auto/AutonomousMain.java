package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.bot.subsystems.ClawArm.ArmState.LOWERED;
import static org.firstinspires.ftc.teamcode.bot.subsystems.ClawArm.ArmState.RAISED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.commands.DriveToProp;
import org.firstinspires.ftc.teamcode.bot.commands.AutoLowerArm;
import org.firstinspires.ftc.teamcode.bot.commands.Delay;
import org.firstinspires.ftc.teamcode.bot.commands.LocalDrive;
import org.firstinspires.ftc.teamcode.bot.commands.MoveArm;
import org.firstinspires.ftc.teamcode.bot.commands.PlacerDrive;
import org.firstinspires.ftc.teamcode.bot.commands.PowerSlide;
import org.firstinspires.ftc.teamcode.bot.commands.RetractLinearSlide;
import org.firstinspires.ftc.teamcode.bot.commands.ToggleIntake;
import org.firstinspires.ftc.teamcode.bot.subsystems.CenterstageVision;
import org.firstinspires.ftc.teamcode.bot.subsystems.ClawArm;
import org.firstinspires.ftc.teamcode.bot.subsystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.bot.subsystems.DualLinearSlide;
import org.firstinspires.ftc.teamcode.bot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bot.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.opencv.pipeline.CenterStagePipeline;
import org.firstinspires.ftc.teamcode.util.CameraStream;
import org.firstinspires.ftc.teamcode.util.PS4GamepadEx;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class AutonomousMain extends LinearOpMode {

    MecanumDriveBase drive;
    DualLinearSlide slide;
    SequentialCommandGroup retractSlide;
    Intake intake;
    ClawArm clawArm;
    AutoLowerArm autoLowerArm;
    MoveArm raiseArm;
    MoveArm lowerArm;
    DroneLauncher droneLauncher;

    CenterstageVision cv;

    DriveToProp driveToProp;

    @Override
    public void runOpMode() {
        drive = new MecanumDriveBase(hardwareMap);
        clawArm = new ClawArm(hardwareMap, "arm1", "claw");
        slide = new DualLinearSlide(hardwareMap, "rightSlide", "leftSlide", 4300);
        intake = new Intake(hardwareMap, "dropdown", "perpendicularEncoder");
        droneLauncher = new DroneLauncher(hardwareMap, "launcher");
        cv = new CenterstageVision(hardwareMap, "Camera");

        retractSlide = new SequentialCommandGroup(
                new InstantCommand(clawArm::open, clawArm),
                new Delay(200),
                new RetractLinearSlide(slide, clawArm)
        );

        autoLowerArm = new AutoLowerArm(slide, clawArm);
        raiseArm = new MoveArm(clawArm, RAISED);
        lowerArm = new MoveArm(clawArm, LOWERED);

        CommandScheduler.getInstance().registerSubsystem(drive);

        clawArm.setDefaultCommand(autoLowerArm);

        telemetry.addData("PropPosition", cv::getPropPosition);

        CommandScheduler.getInstance().schedule(new RunCommand(telemetry::update));

        driveToProp = new DriveToProp(drive, intake, cv);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new Delay(200),
                        driveToProp
                )
        );

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
        }

        CommandScheduler.getInstance().reset();
    }
}
