package org.firstinspires.ftc.teamcode.auto.pipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class 一ConvertToCbChannel extends OpenCvPipeline {
    // Notice this is declared as an instance variable (and re-used), not a local variable
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);


        return Cb;
    }

}
