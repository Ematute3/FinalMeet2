// org/firstinspires/ftc/teamcode/next/kotlin/subsystems/limeLight.kt
package org.firstinspires.ftc.teamcode.next.kotlin.subsystems

import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.next.kotlin.data.Motif
import org.firstinspires.ftc.teamcode.next.subsystems.DriveTrain
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sqrt
import kotlin.math.tan

object limeLight : Subsystem {

    lateinit var ll: Limelight3A

    // ==================== LIMELIGHT CONFIG ====================
    var limelightOn: Boolean = true
    var grabMegaTag = false

    // Distance calculation constants (for Limelight to target tag)
    var llAngle = 9.895942      // Limelight mounting angle in degrees
    var llLensHeight = 10.2756  // Height of limelight lens in inches
    var goalHeight = 29.5       // Height of tag in inches (for distance calc)

    // Auto-alignment settings
    var autoAlignEnabled: Boolean = false
    var targetDistance: Double = 24.0   // Target distance in inches
    var distanceTolerance: Double = 3.0 // Tolerance for distance (inches)
    var angleTolerance: Double = 2.0    // Tolerance for angle alignment (degrees)
    var alignmentKp: Double = 0.02      // Proportional gain for heading correction
    var maxTurnPower: Double = 0.3      // Maximum turn power for alignment

    // Exposed telemetry values
    var angleToGoalDegrees: Double = 0.0
    var angleToGoalRadians: Double = 0.0
    var distanceToGoal: Double? = null
    var currentTx: Double = 0.0
    var currentTy: Double = 0.0
    var currentTa: Double = 0.0
    var hasValidTarget: Boolean = false
    var isAligned: Boolean = false
    var isAtTargetDistance: Boolean = false

    // Motif detection
    var detectedMotif: Motif = Motif.NONE

    // Fiducial data
    var fiducialCount: Int = 0
    var fiducialData: String = "No fiducials"

    // Alignment output
    var alignmentTurnPower: Double = 0.0

    // ==================== SHOOTER PHYSICS (distance -> velocity) ====================

    private const val shooterLaunchAngleDeg = 34.36            // degrees
    private const val shooterGoalHeightIn = 37.85              // inches
    private const val shooterLaunchHeightIn = 329.62843 / 25.4 // mm -> inches ≈ 12.98
    private const val gravityInPerSec2 = 386.09                // in/s^2

    // Shooter wheel geometry (2.83465 is DIAMETER)
    private const val shooterWheelDiameterIn = 2.83465
    private const val shooterWheelRadiusIn = shooterWheelDiameterIn / 2.0

    // Outputs
    var requiredShotSpeedInPerSec: Double? = null
    var requiredShotWheelRpm: Double? = null

    override fun initialize() {
        ll = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "ll")
        ll.setPollRateHz(100)
        ll.pipelineSwitch(0)
        ll.start()
    }

    override fun periodic() {
        if (!limelightOn) {
            clearState()
            return
        }

        val result = ll.latestResult
        if (result == null || !result.isValid) {
            clearState()
            return
        }

        hasValidTarget = true

        updateBasicData(result)
        updateDistanceCalculation(result)
        updateFiducialData(result)
        updateMotif(result)
        updateAlignment()
        updateShotVelocity()  // distance-based shooter velocity
    }

    /** Reset state when LL has no valid data */
    private fun clearState() {
        hasValidTarget = false

        currentTx = 0.0
        currentTy = 0.0
        currentTa = 0.0

        distanceToGoal = null
        angleToGoalDegrees = 0.0
        angleToGoalRadians = 0.0
        isAligned = false
        isAtTargetDistance = false

        fiducialCount = 0
        fiducialData = "No valid result"

        detectedMotif = Motif.NONE
        alignmentTurnPower = 0.0

        requiredShotSpeedInPerSec = null
        requiredShotWheelRpm = null
    }

    // ==================== BASIC LL DATA ====================

    private fun updateBasicData(result: LLResult) {
        currentTx = result.tx
        currentTy = result.ty
        currentTa = result.ta
    }

    // ==================== DISTANCE CALC (TAG) ====================

    private fun updateDistanceCalculation(result: LLResult) {
        val targetOffsetAngleVertical = result.ty

        angleToGoalDegrees = llAngle + targetOffsetAngleVertical
        angleToGoalRadians = angleToGoalDegrees * (PI / 180.0)

        val tanAngle = tan(angleToGoalRadians)

        if (abs(tanAngle) < 1e-3) {
            distanceToGoal = null
            isAtTargetDistance = false
            return
        }

        val dist = (goalHeight - llLensHeight) / tanAngle

        // Reject negative or non-finite distances
        if (!dist.isFinite() || dist <= 0.0) {
            distanceToGoal = null
            isAtTargetDistance = false
            return
        }

        distanceToGoal = dist
        isAtTargetDistance = abs(dist - targetDistance) <= distanceTolerance
    }

    // ==================== ALIGNMENT ====================

    private fun updateAlignment() {
        if (!hasValidTarget || !autoAlignEnabled) {
            alignmentTurnPower = 0.0
            isAligned = false
            return
        }

        isAligned = abs(currentTx) <= angleTolerance

        alignmentTurnPower = if (isAligned) {
            0.0
        } else {
            (-currentTx * alignmentKp).coerceIn(-maxTurnPower, maxTurnPower)
        }
    }

    fun getDistanceToTarget(): Double? = distanceToGoal

    // ==================== FIDUCIALS & MOTIF ====================

    private fun updateFiducialData(result: LLResult) {
        val fiducials = result.fiducialResults
        fiducialCount = fiducials.size

        fiducialData =
            if (fiducials.isNotEmpty()) {
                buildString {
                    for (fr in fiducials) {
                        append("ID: ${fr.fiducialId}, ")
                        append("X: ${"%.2f".format(fr.targetXDegrees)}°, ")
                        append("Strafe: ${"%.2f".format(fr.robotPoseTargetSpace.position.x)}\n")
                    }
                }.trim()
            } else {
                "No fiducials detected"
            }
    }

    private fun updateMotif(result: LLResult) {
        val fR = result.fiducialResults
        detectedMotif =
            if (fR.isNotEmpty()) {
                when (fR[0].fiducialId) {
                    21 -> Motif.GPP
                    22 -> Motif.PGP
                    else -> Motif.PPG
                }
            } else {
                Motif.NONE
            }
    }

    // ==================== MEGATAG POSE ====================

    fun grabResultData(): LLResult? {
        val lR = ll.latestResult
        return if (lR != null && lR.isValid) lR else null
    }

    fun megaTag(): Pose? {
        val lR = ll.latestResult ?: return null

        val yaw = DriveTrain.currentPose.heading
        // If heading is radians and LL expects degrees, convert here:
        // ll.updateRobotOrientation(Math.toDegrees(yaw))
        ll.updateRobotOrientation(yaw)

        val botposeMt2 = lR.botpose_MT2 ?: return null
        return Pose(botposeMt2.position.x, botposeMt2.position.y, yaw)
    }

    // ==================== DISTANCE → SHOOTER VELOCITY ====================

    /**
     * Uses projectile physics with fixed launch angle and heights to compute:
     *  - requiredShotSpeedInPerSec (ball speed, in/s)
     *  - requiredShotWheelRpm (shooter wheel RPM, assuming 1:1 and wheelDiameter)
     *
     * If the geometry doesn't make sense for the current distance, values are null.
     */
    private fun updateShotVelocity() {
        val x = distanceToGoal
        if (x == null || !x.isFinite() || x <= 0.0) {
            requiredShotSpeedInPerSec = null
            requiredShotWheelRpm = null
            return
        }

        val theta = shooterLaunchAngleDeg * (PI / 180.0)
        val cosTheta = cos(theta)
        val tanTheta = tan(theta)
        val deltaY = shooterGoalHeightIn - shooterLaunchHeightIn

        // v^2 = g * x^2 / (2 * cos^2(theta) * (x * tan(theta) - deltaY))
        val denom = 2.0 * cosTheta * cosTheta * (x * tanTheta - deltaY)

        if (denom <= 0.0 || !denom.isFinite()) {
            // Not physically valid for this geometry
            requiredShotSpeedInPerSec = null
            requiredShotWheelRpm = null
            return
        }

        val vSquared = gravityInPerSec2 * x * x / denom
        if (vSquared <= 0.0 || !vSquared.isFinite()) {
            requiredShotSpeedInPerSec = null
            requiredShotWheelRpm = null
            return
        }

        val v = sqrt(vSquared) // in/s
        requiredShotSpeedInPerSec = v

        if (shooterWheelRadiusIn > 0.0) {
            val revPerSec = v / (2.0 * PI * shooterWheelRadiusIn)
            requiredShotWheelRpm = revPerSec * 60.0
        } else {
            requiredShotWheelRpm = null
        }
    }

    /**
     * Optional helper if you want a normalized [0,1] suggestion for motor power.
     * You should set maxShotRpm based on your farthest valid shot in testing.
     */
    var maxShotRpm: Double = 0.0

    fun getSuggestedShooterPower(): Double? {
        val rpm = requiredShotWheelRpm ?: return null
        if (maxShotRpm <= 0.0) return null
        val p = rpm / maxShotRpm
        return if (p.isFinite()) p.coerceIn(0.0, 1.0) else null
    }

    // ==================== COMMANDS ====================

    val enableAutoAlign = InstantCommand {
        autoAlignEnabled = true
    }

    val disableAutoAlign = InstantCommand {
        autoAlignEnabled = false
        alignmentTurnPower = 0.0
    }

    val toggleAutoAlign = InstantCommand {
        autoAlignEnabled = !autoAlignEnabled
        if (!autoAlignEnabled) {
            alignmentTurnPower = 0.0
        }
    }

    val motifCommand = InstantCommand {
        grabResultData()?.let { updateMotif(it) }
    }

    // ==================== TELEMETRY ====================

    fun getTelemetryString(): String {
        return buildString {
            appendLine("=== LIMELIGHT STATUS ===")
            appendLine("Valid Target: $hasValidTarget")
            appendLine("Auto-Align: ${if (autoAlignEnabled) "ENABLED" else "DISABLED"}")
            if (hasValidTarget) {
                appendLine("TX: ${"%.2f".format(currentTx)}°")
                appendLine("TY: ${"%.2f".format(currentTy)}°")
                appendLine("TA: ${"%.2f".format(currentTa)}%")
                appendLine("Angle to Goal (tag): ${"%.2f".format(angleToGoalDegrees)}°")
                appendLine("Distance (tag): ${distanceToGoal?.let { "%.2f".format(it) } ?: "N/A"} in")
                appendLine("At Target Distance: $isAtTargetDistance")
                appendLine("Aligned: $isAligned")
                if (autoAlignEnabled) {
                    appendLine("Turn Power: ${"%.3f".format(alignmentTurnPower)}")
                }
            }
            appendLine("Fiducials: $fiducialCount")
            if (fiducialCount > 0) {
                appendLine(fiducialData)
            }
            appendLine("Motif: $detectedMotif")

            appendLine("--- SHOOTER SOLVER ---")
            appendLine("Shot v: ${requiredShotSpeedInPerSec?.let { "%.1f in/s".format(it) } ?: "N/A"}")
            appendLine("Shot RPM: ${requiredShotWheelRpm?.let { "%.1f rpm".format(it) } ?: "N/A"}")
            appendLine("Suggested Power: ${getSuggestedShooterPower()?.let { "%.2f".format(it) } ?: "N/A"}")
        }
    }

    fun getDistanceDebugInfo(): String {
        return buildString {
            appendLine("=== DISTANCE DEBUG ===")
            appendLine("LL Angle: ${"%.2f".format(llAngle)} deg")
            appendLine("LL Lens Height: ${"%.2f".format(llLensHeight)} in")
            appendLine("Goal Height: ${"%.2f".format(goalHeight)} in")
            appendLine("TY (vertical offset): ${"%.2f".format(currentTy)} deg")
            appendLine("Angle to Goal: ${"%.2f".format(angleToGoalDegrees)} deg / ${"%.3f".format(angleToGoalRadians)} rad")
            appendLine("Distance: ${distanceToGoal?.let { "%.2f".format(it) } ?: "N/A"} in")
        }
    }
}
