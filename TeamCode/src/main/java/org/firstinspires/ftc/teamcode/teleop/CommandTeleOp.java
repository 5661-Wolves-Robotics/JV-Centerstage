package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.bot.commands.AutoLowerArm;
import org.firstinspires.ftc.teamcode.bot.commands.LocalDrive;
import org.firstinspires.ftc.teamcode.bot.commands.PowerSlide;
import org.firstinspires.ftc.teamcode.bot.commands.ToggleIntake;
import org.firstinspires.ftc.teamcode.bot.subsystems.CenterstageVision;
import org.firstinspires.ftc.teamcode.bot.subsystems.ClawArm;
import org.firstinspires.ftc.teamcode.bot.subsystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.bot.subsystems.DualLinearSlide;
import org.firstinspires.ftc.teamcode.bot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.opencv.pipeline.CenterStagePipeline;
import org.firstinspires.ftc.teamcode.util.CameraStream;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class CommandTeleOp extends CommandOpMode {

    GamepadEx driver, placer;

    Mecanum drive;
    LocalDrive localDrive;
    DualLinearSlide slide;
    PowerSlide powerSlide;
    Intake intake;
    ToggleIntake toggleIntake;
    ClawArm clawArm;
    AutoLowerArm autoLowerArm;
    DroneLauncher droneLauncher;

    CenterstageVision cv;

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        placer = new GamepadEx(gamepad2);

        drive = new Mecanum(hardwareMap);
        localDrive = new LocalDrive(drive, driver::getLeftX, driver::getLeftY, driver::getRightX);

        slide = new DualLinearSlide(hardwareMap, "rightSlide", "leftSlide", 4300);
        powerSlide = new PowerSlide(slide, ()-> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        intake = new Intake(hardwareMap, "dropdown", "perpendicularEncoder");
        toggleIntake = new ToggleIntake(intake);

        clawArm = new ClawArm(hardwareMap, "arm1", "claw");
        autoLowerArm = new AutoLowerArm(slide, clawArm);

        droneLauncher = new DroneLauncher(hardwareMap, "launcher");

        cv = new CenterstageVision(hardwareMap, "Camera");

        driver.getGamepadButton(GamepadKeys.Button.X)
                        .whenPressed(toggleIntake);
        driver.getGamepadButton(GamepadKeys.Button.B)
                        .whenPressed(new InstantCommand(intake::reverse, intake))
                        .whenInactive(new ConditionalCommand(
                                new InstantCommand(intake::power, intake),
                                new InstantCommand(intake::disable, intake),
                                ()-> intake.getState() == Intake.IntakeState.LOWERED
                        ));

        placer.getGamepadButton(GamepadKeys.Button.Y)
                        .whenPressed(new InstantCommand(droneLauncher::launch, droneLauncher));

        slide.setDefaultCommand(powerSlide);
        clawArm.setDefaultCommand(autoLowerArm);

        schedule(localDrive);
    }
}
