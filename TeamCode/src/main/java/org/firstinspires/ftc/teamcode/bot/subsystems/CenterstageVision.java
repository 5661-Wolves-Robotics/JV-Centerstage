package org.firstinspires.ftc.teamcode.bot.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opencv.pipeline.CenterStagePipeline;
import org.firstinspires.ftc.teamcode.util.CameraStream;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class CenterstageVision extends SubsystemBase {

    private final WebcamName m_cam;
    private final CenterStagePipeline pipeline;

    private final VisionPortal vision;
    private final AprilTagProcessor aprilTagProcessor;
    private final CameraStream cameraStream;

    private CenterStagePipeline.PropPosition detectedPos = null;

    public CenterstageVision(HardwareMap hardwareMap, String cam){
        m_cam = hardwareMap.get(WebcamName.class, cam);

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        cameraStream = new CameraStream();

        pipeline = new CenterStagePipeline();

        vision = new VisionPortal.Builder()
                .setCamera(m_cam)
                .addProcessor(cameraStream)
                .addProcessor(aprilTagProcessor)
                .addProcessor(pipeline)
                .build();

        FtcDashboard.getInstance().startCameraStream(cameraStream, 0);
    }

    public void storePropPosition(CenterStagePipeline.PropPosition propPos){
        detectedPos = propPos;
    }

    public CenterStagePipeline.PropPosition getPropPosition(){
        return detectedPos;
    }

    public CenterStagePipeline.PropPosition getPipelinePosition(){
        return pipeline.getPosition();
    }

    public List<AprilTagDetection> getAprilTags(){
        return aprilTagProcessor.getDetections();
    }

}
