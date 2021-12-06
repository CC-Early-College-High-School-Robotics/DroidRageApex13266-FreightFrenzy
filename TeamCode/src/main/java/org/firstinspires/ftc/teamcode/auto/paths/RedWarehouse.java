package org.firstinspires.ftc.teamcode.auto.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipeline.DuckDetection;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MechanumDriveRoadRunner;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Red Warehouse (Top) Roadrunner Path", group="Roadrunner Paths")
public class RedWarehouse extends LinearOpMode {
    @Override
    public void runOpMode() {
        MechanumDriveRoadRunner drive = new MechanumDriveRoadRunner(hardwareMap);
        Devices robot = new Devices();
        robot.init(hardwareMap);

        double armHeight = 0;
        double hubDistance = 0;

        DuckDetection detector = new DuckDetection();


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

                camera.setPipeline(detector);
                telemetry.addData("you shoudl see this", "ill be mad if you dont");
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera status", "Camera failed :(");
            }
        });
        sleep(4000);
        // camera dection print
        telemetry.addData("Duck position", detector.getAnalysis());
        telemetry.addData("hi", "hi");

        if (detector.getAnalysis() == DuckDetection.DuckPosition.RIGHT) {
            armHeight = robot.ARM_HIGH_POS;
            hubDistance = 20;
        }

        if (detector.getAnalysis() == DuckDetection.DuckPosition.CENTER) {
            armHeight = robot.ARM_MID_POS;
            hubDistance = 17;
        }

        if (detector.getAnalysis() == DuckDetection.DuckPosition.LEFT) {
            armHeight = robot.ARM_LOW_POS;
            hubDistance = 19.25;

        }
        camera.stopStreaming();









        Pose2d startPose = new Pose2d(-11, -60, Math.toRadians(270));
        ElapsedTime timer = new ElapsedTime();

        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .back(hubDistance)
                .build();

        TrajectorySequence Trajectory2 = drive.trajectorySequenceBuilder(Trajectory1.end())
                .forward(hubDistance)
                .build();

        TrajectorySequence Trajectory3 = drive.trajectorySequenceBuilder(Trajectory2.end())
                .turn(Math.toRadians(80))
                .build();

        TrajectorySequence Trajectory4 = drive.trajectorySequenceBuilder(Trajectory3.end())
                .forward(48)
                .build();

        // Before start

        // Move camera
        robot.cameraServo.setPosition(0.25);

        // Lift box up
        robot.boxServo.setPosition(robot.BOX_FORWARD);

        // Lift Arm
        robot.armMotor.setPower(robot.ARM_POWER);
        robot.setArmPosition(armHeight);




        // On start

        waitForStart();
        if(isStopRequested()) return;



        // Run trajectory 1
        drive.followTrajectorySequence((Trajectory1));

        // Drop freight
        robot.boxServo.setPosition(robot.BOX_DROP);
        sleep(1000);

        // Run trajectory 2
        drive.followTrajectorySequence((Trajectory2));

        // Return box
        robot.boxServo.setPosition(robot.BOX_INTAKE);

        // Run trajectory 3
        drive.followTrajectorySequence((Trajectory3));

        //return arm
        robot.setArmPosition(robot.ARM_INTAKE_POS);

        // Run trajectory 4
        drive.followTrajectorySequence((Trajectory4));
    }
}

