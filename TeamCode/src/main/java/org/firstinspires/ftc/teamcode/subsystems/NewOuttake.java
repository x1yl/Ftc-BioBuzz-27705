package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.hardwares.Flywheel;

import java.util.Collections;
import java.util.List;


public class NewOuttake {

    private static double kP = .1, kI, kD; //PID coeffs, set to private after tuning
    private final Flywheel flywheel;
    private double currentTargetVelocity = 0;
    private double currentVelocity;
    private boolean readyToShoot;
    //coefficents
    private final double kV = 0.00045; //kv should be 1 / maxVelocity from encoder, create a short op mod for that
//    private double kS = .415; //may want to remove ks
    private double kS = 0.245;
    private final Servo hoodServo;
    private final VoltageSensor voltage;
//    private final InterpLUT lut;
    private boolean initalized;
    private boolean atTargetVelocity;

    //notes: 1 PIDFCoefficents for both motors, use the numbers for one for both
    //kS: smallest number to get the motor to spin, overcoming friction
    //kv is amount of power needed to get to max velocity / max velocity, so its 1/max velocity
    //that way if target velo is = max velo, then it will give you the max voltage or power that the motor can use
    //https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html
    //https://www.youtube.com/watch?v=lzVuUpwNdZo
    //https://docs.ftclib.org/ftclib/features/util


    //(kP * (targetVelocity - left.getVelocity()) + kV * targetVelocity) * 12 / volts; volts is optional, voltage sensor
    //

    private static List<LynxModule> allHubs = Collections.emptyList();


    public static void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }


    public NewOuttake(HardwareMap hardwareMap) {
        DcMotorEx flywheelMotorTop = hardwareMap.get(DcMotorEx.class, "flywheelMotor1");
        DcMotorEx flywheelMotorBottom = hardwareMap.get(DcMotorEx.class, "flywheelMotor2");
        voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");
        flywheel = new Flywheel(flywheelMotorTop, flywheelMotorBottom);

//        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hoodServo = hardwareMap.get(Servo.class, "Hood Servo");
        hoodServo.setDirection(Servo.Direction.REVERSE);
        flywheel.getVelocity();

//        lut = new InterpLUT();
////        lut.add(); //1st distance, second target velocity
//        lut.createLUT();

        allHubs = hardwareMap.getAll(LynxModule.class);
        initalized = false;
    }
    //https://github.com/first-tech-challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptMotorBulkRead.java
    public void updatePID() {

        if (!initalized) {
            hoodServo.setPosition(0);
            atTargetVelocity = false;
            currentTargetVelocity = 0;
            initalized = true;
        }
        if (currentTargetVelocity == 0) {
            flywheel.setPower(0);
            return;
        }
        clearBulkCache();

        currentVelocity = flywheel.getVelocity(); // max vel in ticks per second should be 2800
        double error = currentTargetVelocity - currentVelocity;
        double feedback = error * kP;
        double feedforward = kV * currentTargetVelocity + kS;

        atTargetVelocity = error == 0 || Math.abs(error - 20) == 0;
        clearBulkCache();


        double pidOutput = Math.max(-1.0, Math.min(1,feedback + feedforward * 12/ voltage.getVoltage() ));
        //prob want to implement some deadzone check for error and break out of updatePID while returning
        flywheel.setPower( pidOutput);
    }

    public String outtakeLog () {
        if (isAtTargetVelocity()) {
            return "flywheel ready";
        }
        return "Flywheel not ready";
    }

    public boolean isAtTargetVelocity () {
        return atTargetVelocity;
    }

    public void setTargetVel (int vel) {
        currentTargetVelocity = vel;
    }

    //https://docs.ftclib.org/ftclib/features/util#what-is-a-look-up-table
    //no need to use this in auto, just do 1 pos, only use for teleop
    //check if shooting mode active in teleop loop then run limelight, autoUpdateTargetVel and feed limelight pos
    //get vectorDistance from limelight instead of from odo pods
//    public void autoUpdateTargetVel(double distance) {
//        currentTargetVelocity = lut.get(distance);
//    }

    //TODO: Implement small InterpLUT for hood
    public void autoUpdateHood(Pose2d vectorDistance) {
        double distance = Math.sqrt(Math.pow(vectorDistance.component1().x, 2)
                + Math.pow(vectorDistance.component1().y, 2));
        if (distance <= 10) { //def incorrect fix later, only should have two or angles tho to reduce time to test and variability
            setHoodAngle(.1);
        }
        if (distance > 40) {
            setHoodAngle(.5);
        }
    }

    public double getHoodAngle() {
        return hoodServo.getPosition();
    }

    public void setHoodAngle(double angle) {
        // Clamp between safe mechanical limits (0.4 to 0.8) to prevent hood from dropping too low
        double clampedAngle = Math.max(0.4, Math.min(0.8, angle));
        hoodServo.setPosition(clampedAngle);
    }

    @SuppressLint("DefaultLocale")
    public String log() {
        return String.format(
                "currentTargetVelocity: %.2f, Current Velocity: %.2f, Current Hood Angle: %.2f",
                currentTargetVelocity,
                currentVelocity,
                hoodServo.getPosition()
        );
    }

    public boolean isAtTarget() {
        return atTargetVelocity;
    }

    //https://youtu.be/aPNCpZzCTKg?t=786
    @TeleOp(name = "FlyWheel Tuner", group = "test")
    public static class FlyWheelTuner extends OpMode {
        @Config
        public static class flywheeltunerconstants {
            public static double targetVelocity;
            public static double P, V, S; //do not need kI or kD , find starting number
        }
        public static double velocity;
        private Flywheel flywheel;
        private Servo hoodServo;
        private VoltageSensor vs;

        private boolean init;


        //hood angles: Low = ,High =

        @Override

        // TODO: ETHAN READ THIS -> Here's how you tune it IN THE CORRECT ORDER
        //
        //kS stands for "static." It's the greatest power you can give the flywheel motor(s) without it moving.
        //So it should be a really small number below like 0.05 or something.
        //You should increase it until it begins to move and then decrease it until it stops moving again.
        //
        // kV is the next number you tune. Pick a velocity like 1500 or something and then increase kV until velocity gets there without going past target.
        //Tune P. Send a ball through the launcher and increase P until it doesn't overshoot past the target velocity in the recovery phase.
        // Keep increasing though because unless you see overshoot you're fine.
        //Tune I. After all the other numbers are tuned, save them, and then make kS slightly negative.
        // This will simulate a drop in power to the motor from a low battery. Increase I until your confident that you can still get to your target velocity.
        //DO ALL OF THIS ON FTCDashboard
        //

        public void init() {
            DcMotorEx flywheelMotorTop = hardwareMap.get(DcMotorEx.class, "flywheelMotor1");
            DcMotorEx flywheelMotorBottom = hardwareMap.get(DcMotorEx.class, "flywheelMotor2");

            flywheelMotorTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheelMotorBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            vs = hardwareMap.get(VoltageSensor.class, "Control Hub");

            flywheel = new Flywheel(flywheelMotorTop, flywheelMotorBottom);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            hoodServo = hardwareMap.get(Servo.class, "Hood Servo");

            hoodServo.setDirection(Servo.Direction.REVERSE);
            init = false;
//            for (LynxModule hub : allHubs) {
//                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//            }
//            clearBulkCache();
        }

        @Override
        public void loop() {
            if (!init) {
                hoodServo.setPosition(1);
                init = true;
            }
//            clearBulkCache();
            velocity = flywheel.getVelocity();
            double volts = vs.getVoltage();

            telemetry.addData("TargetVel: ", flywheeltunerconstants.targetVelocity);
            telemetry.addData("CurrentVel: ", velocity);
            double error = flywheeltunerconstants.targetVelocity - velocity;
            double feedback = error * flywheeltunerconstants.P;
            double feedforward = flywheeltunerconstants.V * flywheeltunerconstants.targetVelocity + flywheeltunerconstants.S;
            double pidOutput = Math.max(-1.0, Math.min(1,feedback + feedforward) * (12/volts));
            flywheel.setPower(pidOutput);
            telemetry.addData("pidOutput: ", pidOutput);
            telemetry.addData("Voltage sensor reading: ", volts);
        }
    }

    @TeleOp(name = "Flywheel Distance Tests", group = "Testing")
    public static class DistanceTests extends LinearOpMode {
        DcMotorEx flywheelTop = hardwareMap.get(DcMotorEx.class, "FlywheelTop");
        DcMotorEx flywheelBot = hardwareMap.get(DcMotorEx.class, "FlywheelBot");
        Flywheel flywheel = new Flywheel(flywheelTop, flywheelBot);
        double flywheelTargetVelocity = 1500;
        Pose2d position = new Pose2d(0,0,0);

        Telemetry telemetry;
        TelemetryPacket tPacket;

        MecanumDrive driveTrain;

        @Override
        public void runOpMode() {
            waitForStart();

            while(opModeInInit()) {
                telemetry.addLine("Position the robot with the intake against the goal.");
                telemetry.update();
            }

            driveTrain = new MecanumDrive(this.hardwareMap, position);

            while(!opModeIsActive()) {

                if(gamepad1.startWasReleased()) {
                    TrajectoryActionBuilder goForwards = driveTrain.actionBuilder(position)
                            .lineToX(position.position.x - 5); // move 5 inches forward, adjust as needed based on testing

                    gamepad1.setLedColor(1,0,0, 10000);
                    Actions.runBlocking(goForwards.build());
                    position = new Pose2d(position.position.x - 5, position.position.y, 0);
                    gamepad1.setLedColor(0,1,0, 10000);
                }

                if(gamepad1.backWasReleased()) {
                    TrajectoryActionBuilder goBackwards = driveTrain.actionBuilder(position)
                            .lineToX(position.position.x + 5); // move 5 inches forward, adjust as needed based on testing

                    gamepad1.setLedColor(1,0,0, 10000);
                    Actions.runBlocking(goBackwards.build());
                    position = new Pose2d(position.position.x + 5, position.position.y, 0);
                    gamepad1.setLedColor(0,1,0, 10000);
                }

                if(gamepad1.dpadUpWasReleased()){
                    flywheelTargetVelocity += 100;
                    flywheel.setVelocity(flywheelTargetVelocity);
                }
                if(gamepad1.dpadDownWasReleased()) {
                    flywheelTargetVelocity -= 100;
                    flywheel.setVelocity(flywheelTargetVelocity);
                }

                telemetry.addData("Current Dist From Goal", position.position.x);
                telemetry.addData("Flywheel Target Velocity", flywheelTargetVelocity);
                telemetry.update();
            }
        }
    }
}