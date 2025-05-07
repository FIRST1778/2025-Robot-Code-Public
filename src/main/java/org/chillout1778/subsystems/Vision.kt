package org.chillout1778.subsystems

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants
import org.chillout1778.Utils.isInsideField
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.estimation.TargetModel
import org.photonvision.targeting.PhotonPipelineResult
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt

object Vision: SubsystemBase(){
    class Camera(initialName: String, robotToCamera: Transform3d) : PhotonCamera(initialName) {
        val poseEstimator = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(Constants.Vision.FIELD_TYPE),
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera
        ).apply {
            tagModel = TargetModel.kAprilTag36h11
            setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)
        }
    }

    private val cameras = arrayOf(
        Camera(Constants.Vision.FRONT_RIGHT_NAME, Constants.Vision.FRONT_RIGHT_TRANSFORM),
        Camera(Constants.Vision.FRONT_LEFT_NAME, Constants.Vision.FRONT_LEFT_TRANSFORM),
        Camera(Constants.Vision.BACK_RIGHT_NAME, Constants.Vision.BACK_RIGHT_TRANSFORM),
        Camera(Constants.Vision.BACK_LEFT_NAME, Constants.Vision.BACK_LEFT_TRANSFORM)
    )

    fun allConnected() = cameras.all { it.isConnected() }

    fun removeResult(res: PhotonPipelineResult): Boolean {
        return res.getTargets().map { it.fiducialId }.any { it in listOf(4, 5, 14, 15) }
    }

    fun periodicAddMeasurements(estimator: SwerveDrivePoseEstimator) {
        for (camera in cameras) {
            val results = camera.getAllUnreadResults().filterNot {
                removeResult(it)
            }
            val estimatedPoses = results.mapNotNull { camera.poseEstimator.update(it).getOrNull() }
            for (pose in estimatedPoses) {
                if(pose.estimatedPose.translation.toTranslation2d().isInsideField()
                    && (pose.strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR ||
                            (pose.strategy == PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
                                    && pose.targetsUsed[0].poseAmbiguity < 0.05
                                    && pose.targetsUsed[0].area > 0.25
                                    )
                            )
                ){

//                    updateStdev(pose.estimatedPose.toPose2d())
                    estimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds)
                }
            }
        }
    }

//    //region standard deviation tracking
//    var stIndex = 0
//    var stPoses = Array(50) {Pose2d()}
//    var xStdev = 0.0
//    var yStdev = 0.0
//    var thetaStdev = 0.0
//    fun updateStdev(newPose: Pose2d) {
//        stPoses[stIndex] = newPose
//        stIndex = (stIndex+1) % stPoses.size
//
//        fun stdev(xs: List<Double>): Double {
//            val mean = xs.sum() / xs.size
//            val variance = xs.sumOf { (it - mean) * (it - mean) } / (xs.size-1)
//
//            return sqrt(variance)
//        }
//        xStdev = stdev(stPoses.map { it.translation.x })
//        yStdev = stdev(stPoses.map { it.translation.y })
//
//        val thetaMean = Rotation2d.fromRadians(atan2(
//            stPoses.map{it.rotation.sin}.sum(),
//            stPoses.map{it.rotation.cos}.sum()
//        ))
//        fun distRadians(r1: Rotation2d, r2: Rotation2d) = abs((r1-r2).radians)
//        val thetaVariance = stPoses.map{distRadians(it.rotation, thetaMean) * distRadians(it.rotation, thetaMean)}.sum() / (stPoses.size-1)
//        thetaStdev = sqrt(thetaVariance)
//    }
    //endregion

    override fun initSendable(builder: SendableBuilder) {
//        builder.addDoubleProperty("X standard deviation", {xStdev}, {})
//        builder.addDoubleProperty("Y standard deviation", {yStdev}, {})
//        builder.addDoubleProperty("Theta standard deviation", {thetaStdev}, {})
        for(camera in cameras){
            builder.addBooleanProperty(camera.name + " connection status", {camera.isConnected}, {})
        }
    }
}