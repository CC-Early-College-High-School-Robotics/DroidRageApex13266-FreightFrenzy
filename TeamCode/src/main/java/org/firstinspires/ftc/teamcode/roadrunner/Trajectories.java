package org.firstinspires.ftc.teamcode.roadrunner;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadrunnerDriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadrunnerDriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadrunnerDriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadrunnerDriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;

import java.util.Arrays;

@Config
public class Trajectories {
    public static double warehouseInSpeed = 0.5;
    public static double splineInSpeed = 0.5;
    public static double warehouseOutSpeed = 0.5;
    public static double splineOutSpeed = 0.5;

    public static MinVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(MAX_ANG_VEL),
            new TankVelocityConstraint(MAX_VEL, TRACK_WIDTH)
    ));
    public static MinVelocityConstraint warehouseInConstraint = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(MAX_ANG_VEL),
            new TankVelocityConstraint(MAX_VEL * warehouseInSpeed, TRACK_WIDTH)
    ));    public static MinVelocityConstraint splineInConstraint = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(MAX_ANG_VEL),
            new TankVelocityConstraint(MAX_VEL * splineInSpeed, TRACK_WIDTH)
    ));
    public static MinVelocityConstraint warehouseOutConstraint = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(MAX_ANG_VEL),
            new TankVelocityConstraint(MAX_VEL * warehouseOutSpeed, TRACK_WIDTH)
    ));
    public static MinVelocityConstraint splineOutConstraint = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(MAX_ANG_VEL),
            new TankVelocityConstraint(MAX_VEL * splineOutSpeed, TRACK_WIDTH)
    ));


    public static ProfileAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL);

    @Config
    public static class WarehouseRed {
        public static Pose2d startPose = new Pose2d(-62.5, 52, Math.toRadians(180));

        public static Pose2d shippingHub = new Pose2d (0,0);

        public static double highGoalX = -3;
        public static double highGoalY = 39;
        public static double intakeFirst = 36;
        public static double intakeDistance = 12;
        public static double shootMoreDistance = 24;
        public static double wobbleDistance = 9;
        public static double wobbleAngle = 226;
    }
    @Config
    public static class WarehouseBlue {
        public static Pose2d startPose = new Pose2d(-62.5, 52, Math.toRadians(180));
        public static double shootDistance = 60;
        public static double wobbleGoalSquareDistance = 48;

        public static double ringX = -30;
        public static double ringY = 32;
        public static double intakeFirst = 36;
        public static double intakeDistance = 12;
        public static double shootMoreDistance = 24;
        public static double wobbleDistance = 9.25;
        public static double wobbleAngle = 211;
    }
    @Config
    public static class CarouselRed {
        public static Pose2d startPose = new Pose2d(-62.5, 52, Math.toRadians(180));

        public static double wobbleGoalSquareDistance = 84;
        public static double wobbleGoalX = 20;

        public static double wobbleGoalY = 30;

        public static double highGoalX = -3;
        public static double highGoalY = 38;
        public static double intakeFirst = 36;
        public static double intakeDistance = 12;
        public static double shootMoreDistance = 24;
        public static double wobbleDistance = 26;
        public static double wobbleAngle = 195;
    }
    @Config
    public static class CarouselBlue {
        public static Pose2d startPose = new Pose2d(-62.5, 52, Math.toRadians(180));

        public static double wobbleGoalSquareDistance = 84;
        public static double wobbleGoalX = 20;

        public static double wobbleGoalY = 30;

        public static double highGoalX = -3;
        public static double highGoalY = 38;
        public static double intakeFirst = 36;
        public static double intakeDistance = 12;
        public static double shootMoreDistance = 24;
        public static double wobbleDistance = 26;
        public static double wobbleAngle = 195;
    }
}