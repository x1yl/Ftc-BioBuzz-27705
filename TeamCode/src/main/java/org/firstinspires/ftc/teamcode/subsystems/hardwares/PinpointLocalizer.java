package org.firstinspires.ftc.teamcode.subsystems.hardwares;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;

import java.util.Objects;

@Config
public final class
PinpointLocalizer implements Localizer {
    public static class Params {

        //note I flipped the odometry pods on the robot so x is now y and y is now x
        public double parYTicks = -3995.6810984724534; // y position of the parallel encoder (in tick units) //may have to be reversed
        public double perpXTicks = 764.0862915219022; // x position of the perpendicular encoder (in tick units) //may have to be reversed
        //try to manually calculate these
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot;

    //Field is inverted
    public PinpointLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        // TODO: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        double mmPerTick = inPerTick * 25.4;
        driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        driver.setOffsets(222.25, 215.9, DistanceUnit.MM); //-222.25 for x?
        driver.resetPosAndIMU();
        driver.recalibrateIMU();


        // TODO: reverse encoder directions if needed
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD; //y reverse
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD; //x forward

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

//        driver.setPosition(new Pose2D(DistanceUnit.INCH, initialPose.component1().x,
//                initialPose.component1().y,
//                AngleUnit.RADIANS, initialPose.component2().toDouble())); //driver position has to be set to initalPose

        txWorldPinpoint = initialPose;
        txPinpointRobot = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    @Override
    public PoseVelocity2d update() {
        driver.update();
//note: mod everything by 360 for all unnormalized angle unit mod by 2 pi
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), driver.getHeading(UnnormalizedAngleUnit.RADIANS));
            Vector2d worldVelocity = new Vector2d(driver.getVelX(DistanceUnit.INCH), driver.getVelY(DistanceUnit.INCH));
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);

            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }
}
