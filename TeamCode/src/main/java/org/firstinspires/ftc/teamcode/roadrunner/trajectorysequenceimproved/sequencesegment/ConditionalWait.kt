package org.firstinspires.ftc.teamcode.roadrunner.trajectorysequenceimproved.sequencesegment

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequenceimproved.sequencesegment.SequenceSegment

class ConditionalWait(startPose: Pose2d, markers: List<TrajectoryMarker>, var condition: () -> Boolean) : SequenceSegment(
    { 0.0 },
    startPose,
    startPose,
    markers
)