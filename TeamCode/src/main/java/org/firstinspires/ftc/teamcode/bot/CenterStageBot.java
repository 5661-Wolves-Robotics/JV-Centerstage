package org.firstinspires.ftc.teamcode.bot;

import static org.firstinspires.ftc.teamcode.bot.subsystems.ClawArm.ArmState.LOWERED;
import static org.firstinspires.ftc.teamcode.bot.subsystems.ClawArm.ArmState.RAISED;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.bot.commands.AutoLowerArm;
import org.firstinspires.ftc.teamcode.bot.commands.MoveArm;
import org.firstinspires.ftc.teamcode.bot.commands.RetractLinearSlide;
import org.firstinspires.ftc.teamcode.bot.subsystems.CenterstageVision;
import org.firstinspires.ftc.teamcode.bot.subsystems.ClawArm;
import org.firstinspires.ftc.teamcode.bot.subsystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.bot.subsystems.DualLinearSlide;
import org.firstinspires.ftc.teamcode.bot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bot.subsystems.MecanumDriveBase;

@Config
public class CenterStageBot extends Robot {

    public MecanumDriveBase drive;
    public ClawArm clawArm;
    public DualLinearSlide slide;
    public Intake intake;
    public DroneLauncher droneLauncher;
    public CenterstageVision cv;

    public SequentialCommandGroup retractSlide;
    public AutoLowerArm autoLowerArm;
    public MoveArm lowerArm;
    public MoveArm raiseArm;

    public CenterStageBot(HardwareMap hardwareMap){
        drive = new MecanumDriveBase(hardwareMap);
        clawArm = new ClawArm(hardwareMap, "arm1", "claw");
        slide = new DualLinearSlide(hardwareMap, "rightSlide", "leftSlide", 4300);
        intake = new Intake(hardwareMap, "dropdown", "perpendicularEncoder");
        droneLauncher = new DroneLauncher(hardwareMap, "launcher");

        cv = new CenterstageVision(hardwareMap, "Camera");

        retractSlide = new SequentialCommandGroup(
                new InstantCommand(clawArm::open, clawArm).withTimeout(200),
                new RetractLinearSlide(slide, clawArm)
        );

        autoLowerArm = new AutoLowerArm(slide, clawArm);
        raiseArm = new MoveArm(clawArm, RAISED);
        lowerArm = new MoveArm(clawArm, LOWERED);

        register(drive);
        clawArm.setDefaultCommand(autoLowerArm);
    }

}
