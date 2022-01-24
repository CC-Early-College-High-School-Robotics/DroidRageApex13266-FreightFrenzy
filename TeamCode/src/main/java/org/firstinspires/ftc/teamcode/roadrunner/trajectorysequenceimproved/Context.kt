package org.firstinspires.ftc.teamcode.roadrunner.trajectorysequenceimproved

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.roadrunner.roadrunnerext.polarAdd
import org.firstinspires.ftc.teamcode.roadrunner.roadrunnerext.toPose

@Config
object Context {
    @JvmField
    var robotPose = Pose2d()
    @JvmField
    var poseVelocity = Pose2d()
    @JvmField
    var packet = TelemetryPacket()
    @JvmField
    var telemetry: Telemetry? = null
    val futurePose: Pose2d
        get() = robotPose
            .polarAdd(poseVelocity.x)
            .vec()
            .polarAdd(poseVelocity.y, robotPose.heading + Math.PI)
            .toPose(robotPose.heading)
}