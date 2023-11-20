package org.firstinspires.ftc.teamcode.opencv.pipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CenterStagePipeline extends OpenCvPipeline {

    public enum PropPosition {
        LEFT,
        RIGHT,
        CENTER
    }

    private final Scalar LINE_COL = new Scalar(0, 255, 0);

    private final Point LEFT_RECT_P = new Point(180, 220);
    private final Point CENTER_RECT_P = new Point(320, 160);
    private final Point RIGHT_RECT_P = new Point(460, 220);

    private final Rect LEFT_RECT = new Rect(
        LEFT_RECT_P,
        new Size(20, 20)
    );
    private final Rect CENTER_RECT = new Rect(
        CENTER_RECT_P,
        new Size(20, 20)
    );
    private final Rect RIGHT_RECT = new Rect(
        RIGHT_RECT_P,
        new Size(20, 20)
    );

    private volatile PropPosition propPos = PropPosition.LEFT;

    private Mat frame = new Mat();
    private Mat cb = new Mat();
    private Mat cr = new Mat();

    private Mat leftCb, rightCb, centerCb;

    private void toYCrCb(Mat rgbFrame){
        Imgproc.cvtColor(rgbFrame, frame, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(frame, cb, 2);
        Core.extractChannel(frame, cr, 1);
    }

    @Override
    public void init(Mat rgbFrame) {
        toYCrCb(rgbFrame);

        leftCb = cb.submat(LEFT_RECT);
        rightCb = cb.submat(RIGHT_RECT);
        centerCb = cb.submat(CENTER_RECT);
    }

    @Override
    public Mat processFrame(Mat input) {

        toYCrCb(input);

        int leftAvg = (int)Core.mean(leftCb).val[0];
        int centerAvg = (int)Core.mean(centerCb).val[0];
        int rightAvg = (int)Core.mean(rightCb).val[0];

        Imgproc.rectangle(input, LEFT_RECT, LINE_COL, 2);
        Imgproc.rectangle(input, CENTER_RECT, LINE_COL, 2);
        Imgproc.rectangle(input, RIGHT_RECT, LINE_COL, 2);

        int max = Math.max(Math.max(leftAvg, rightAvg), centerAvg);

        if(max == leftAvg){
            propPos = PropPosition.LEFT;
            Imgproc.rectangle(input, LEFT_RECT, LINE_COL, -1);
        } else if(max == centerAvg){
            propPos = PropPosition.CENTER;
            Imgproc.rectangle(input, CENTER_RECT, LINE_COL, -1);
        } else {
            propPos = PropPosition.RIGHT;
            Imgproc.rectangle(input, RIGHT_RECT, LINE_COL, -1);
        }

        return input;
    }

    public PropPosition getPosition(){
        return propPos;
    }
}
