package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipeline.BlueCarouselDuckDetection;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="Vision Box Position Programmer", group="test")
public class VisionBoxPositionProgrammer extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Tutorial: ", "Hold x, a, or b with a dpad to move the boxes and find the optimal position.");

        double region1x = 400;
        double region1y = 400;
        double region2x = 400;
        double region2y = 400;
        double x3 = 400;
        double y3 = 400;

        /* Open CV */

        // Obtain camera id to allow for camera preview
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Obtain webcam name
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Initialize OpenCvWebcam
        // With live preview
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);

        // Open the Camera Device Asynchronously
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start Camera Streaming

                // NOTE: this must be called *before* you call startStreaming(...)
                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

                // Start camera stream with 1280x720 resolution
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);

                camera.setPipeline(new BlueCarouselDuckDetection());
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera status", "Camera failed :(");
            }
        });
        while(!isStopRequested()) {
            if(gamepad1.dpad_up && gamepad1.x) {
                region1y -= 0.05;
            }
            else if(gamepad1.dpad_up && gamepad1.a) {
                region2y -= 0.05;
            }
            else if(gamepad1.dpad_up && gamepad1.b) {
                y3 -= 0.05;
            }
            else if(gamepad1.dpad_right && gamepad1.x) {
                region1x += 0.05;
            }
            else if(gamepad1.dpad_right && gamepad1.a) {
                region2x += 0.05;
            }
            else if(gamepad1.dpad_right && gamepad1.b) {
                x3 += 0.05;
            }
            else if(gamepad1.dpad_down && gamepad1.x) {
                region1y += 0.05;
            }
            else if(gamepad1.dpad_down && gamepad1.a) {
                region2y += 0.05;
            }
            else if(gamepad1.dpad_down && gamepad1.b) {
                y3 += 0.05;
            }
            else if(gamepad1.dpad_left && gamepad1.x) {
                region1x -= 0.05;
            }
            else if(gamepad1.dpad_left && gamepad1.a) {
                region2x -= 0.05;
            }
            else if(gamepad1.dpad_left && gamepad1.b) {
                x3 -= 0.05;
            }


            Point Region1 = new Point(region1x, region1y);
            Point Region2 = new Point(region2x, region2y);
            Point Region3 = new Point(x3, y3);
            camera.setPipeline(new BlueCarouselDuckDetection());
            String Region1value = region1x + ", " + region1y;
            String Region2value = region2x + ", " + region2y;
            String Region3value = x3 + ", " + y3;

            telemetry.addData("Box 1 Position: ", Region1value);
            telemetry.addData("Box 2 Position: ", Region2value);
            telemetry.addData("Box 3 Position: ", Region3value);

        }
        waitForStart();
        telemetry.addData("umm", "you cant see the camera stream on a connected driver hub or phone after pressing start");

    }
}
