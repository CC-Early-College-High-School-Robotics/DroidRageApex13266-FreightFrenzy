package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipeline.ConvertToCbChannel;
import org.firstinspires.ftc.teamcode.auto.pipeline.DuckDetection;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="Box Position Programmer", group="test")
public class BoxPositionProgrammer extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Tutorial: ", "Hold x, a, or b with a dpad to move the boxes and find the optimal position.");

        double x1 = 400;
        double y1 = 400;
        double x2 = 400;
        double y2 = 400;
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

                camera.setPipeline(new DuckDetection());
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera status", "Camera failed :(");
            }
        });
        while(!isStopRequested()) {
            if(gamepad1.dpad_up && gamepad1.x) {
                y1 -= 0.05;
            }
            else if(gamepad1.dpad_up && gamepad1.a) {
                y2 -= 0.05;
            }
            else if(gamepad1.dpad_up && gamepad1.b) {
                y3 -= 0.05;
            }
            else if(gamepad1.dpad_right && gamepad1.x) {
                x1 += 0.05;
            }
            else if(gamepad1.dpad_right && gamepad1.a) {
                x2 += 0.05;
            }
            else if(gamepad1.dpad_right && gamepad1.b) {
                x3 += 0.05;
            }
            else if(gamepad1.dpad_down && gamepad1.x) {
                y1 += 0.05;
            }
            else if(gamepad1.dpad_down && gamepad1.a) {
                y2 += 0.05;
            }
            else if(gamepad1.dpad_down && gamepad1.b) {
                y3 += 0.01;
            }
            else if(gamepad1.dpad_left && gamepad1.x) {
                x1 -= 0.05;
            }
            else if(gamepad1.dpad_left && gamepad1.a) {
                x2 -= 0.05;
            }
            else if(gamepad1.dpad_left && gamepad1.b) {
                x3 -= 0.05;
            }


            Point Region1 = new Point(x1, y1);
            Point Region2 = new Point(x2, y2);
            Point Region3 = new Point(x3, y3);
            camera.setPipeline(new DuckDetection(Region1, Region2, Region3));
            String Region1value = x1 + ", " + y1;
            String Region2value = x2 + ", " + y2;
            String Region3value = x3 + ", " + y3;

            telemetry.addData("Box 1 Position: ", Region1value);
            telemetry.addData("Box 2 Position: ", Region2value);
            telemetry.addData("Box 3 Position: ", Region3value);

        }
        waitForStart();
        telemetry.addData("umm", "you cant see the camera stream on a connected driver hub or phone after pressing start");

    }
}
