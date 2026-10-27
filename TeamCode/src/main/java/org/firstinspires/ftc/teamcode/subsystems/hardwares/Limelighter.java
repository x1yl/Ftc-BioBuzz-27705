package org.firstinspires.ftc.teamcode.subsystems.hardwares;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.HashMap;

public class Limelighter {
    // https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming

    public final HashMap<Integer, Pose3D> tagPositions = new HashMap<Integer, Pose3D>();
    private final Limelight3A limelight;

    public Limelighter(HardwareMap hardwareMap) {
        if (hardwareMap == null) {
            throw new IllegalArgumentException("HardwareMap cannot be null!");
        }
        Limelight3A tempLimelight = null;
        try {
            tempLimelight = hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception e1) {
            try {
                tempLimelight = hardwareMap.get(Limelight3A.class, "Limelight");
            } catch (Exception e2) {
                try {
                    tempLimelight = hardwareMap.get(Limelight3A.class, "limelight3a");
                } catch (Exception e3) {
                    throw new IllegalArgumentException("Unable to find Limelight device in HardwareMap (tried 'limelight', 'Limelight', 'limelight3a')");
                }
            }
        }
        limelight = tempLimelight;
        limelight.setPollRateHz(100);
        limelight.start();
        switchPipeline(Pipelines.APRILTAGGER);
        // init known tag positions, from https://www.chiefdelphi.com/t/decode-field-map/506135/2
        tagPositions.put(20, new Pose3D(
                new Position(DistanceUnit.METER, -1.482, -1.413, 0.749, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, 54, 0, 0, 0)
        ));
        tagPositions.put(24, new Pose3D(
                new Position(DistanceUnit.METER, -1.482, 1.413, 0.749, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, -54, 0, 0, 0)
        ));
    }

    @NonNull
    private static Pose2d getPose2d(LLResult result, double robotYaw)
            throws MT2FailedException, MT2ZeroedPosException {
        Pose3D botpose_mt2 = result.getBotpose_MT2();
        if (botpose_mt2 == null) {
            throw new MT2FailedException("Limelight failed to calculate botpose_mt2!");
        }
        double rawX = botpose_mt2.getPosition().x;
        double rawY = botpose_mt2.getPosition().y;

        if (rawX == 0.0 && rawY == 0.0) {
            throw new MT2ZeroedPosException(
                    "Limelight botpose_mt2 position is zeroed, the limelight wasn't confident in " +
                            "its calculation and returned 0.0 for both X and Y."
            );
        }

        // convert Limelight Pose3D (meters) to Roadrunner Pose2d (inches)
        final double METERS_TO_INCHES = 39.3700787;
        double x_in = rawX * METERS_TO_INCHES;
        double y_in = rawY * METERS_TO_INCHES;

        // finalize calculation, return pose
        return new Pose2d(new Vector2d(x_in, y_in), robotYaw);
    }

    public void startLimelight() throws IllegalStateException {
        if (limelight == null) {
            throw new IllegalStateException("Limelight hardware not initialized!");
        }
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public void stopLimelight() throws IllegalStateException {
        if (limelight == null) {
            throw new IllegalStateException("Limelight hardware not initialized!");
        }
        limelight.stop();
    }

    public void switchPipeline(Pipelines pipeline) throws IllegalArgumentException {
        if (pipeline == null) {
            throw new IllegalArgumentException("Pipeline cannot be null!");
        }
        if (pipeline.getValue() < 0 || pipeline.getValue() > 2) {
            throw new IllegalArgumentException("Invalid pipeline value: " + pipeline.getValue());
        }
        limelight.pipelineSwitch(pipeline.getValue());
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public LLStatus getStatus() {
        return limelight.getStatus();
    }

    public Pose2d getRobotPoseFromAprilTag(double yaw)
            throws NoAprilTagException, MT2FailedException, MT2ZeroedPosException {
        limelight.start();
        switchPipeline(Pipelines.APRILTAGGER);

        try {
            // give limelight the yaw so it can do its thing
            limelight.updateRobotOrientation(Math.toDegrees(yaw));

            // get data after orientation update
            LLResult result = getLatestResult();

            if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
                throw new NoAprilTagException("No AprilTags detected!");
            }

            return getPose2d(result, yaw);
        } finally {
            // Always stop to avoid keeping the camera active after a query.
            limelight.stop();
        }
    }

    public Pose2d getRobotVectorFromAprilTag(double yaw)
            throws NoAprilTagException, MT2FailedException, MT2ZeroedPosException {
        // first get a fiducial result
        LLResult result = getLatestResult();
        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
            throw new NoAprilTagException("No AprilTags detected!");
        }
        int tagID = result.getFiducialResults().get(0).getFiducialId();

        // get bot pose and tag pose
        Pose2d calculatedPose = getPose2d(result, yaw);
        Pose3D tagPose3D = tagPositions.get(tagID);
        if (tagPose3D == null) {
            throw new NoAprilTagException("Detected AprilTag ID " + tagID + " does not have a " +
                    "known position!");
        }
        Pose2d tagPose2D = new Pose2d(
                new Vector2d(tagPose3D.getPosition().x, tagPose3D.getPosition().y),
                tagPose3D.getOrientation().getYaw(AngleUnit.RADIANS)
        );

        // calculate delta
        return new Pose2d(
                new Vector2d(
                        calculatedPose.position.x - tagPose2D.position.x,
                        calculatedPose.position.y - tagPose2D.position.y
                ),
                calculatedPose.heading.minus(tagPose2D.heading)
        );
    }

    public double getRobotDistanceFromAprilTag(double yaw)
            throws NoAprilTagException, MT2FailedException, MT2ZeroedPosException {
        Pose2d vectorToTag = getRobotVectorFromAprilTag(yaw);
        return Math.hypot(vectorToTag.position.x, vectorToTag.position.y);
    }

    public int getLatestAprilTagID() throws NoAprilTagException {
        LLResult result = getLatestResult();
        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
            throw new NoAprilTagException("No AprilTags detected!");
        }
        return result.getFiducialResults().get(0).getFiducialId();
    }

    public enum Pipelines {
        APRILTAGGER(0),
        PURPLER(1),
        GREENER(2);

        private final int value;

        Pipelines(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public static class NoAprilTagException extends Exception {
        public NoAprilTagException(String message) {
            super(message);
        }
    }

    public static class MT2FailedException extends Exception {
        public MT2FailedException(String message) {
            super(message);
        }
    }

    public static class MT2ZeroedPosException extends Exception {
        public MT2ZeroedPosException(String message) {
            super(message);
        }
    }
}