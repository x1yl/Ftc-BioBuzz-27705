package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.NewOuttake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.hardwares.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.hardwares.Limelighter;

import java.util.List;

/**
 * SimpleMoveTeleOp provides robot driving, intake, spindexer, hood adjustment,
 * velocity-controlled flywheel shooter presets, and live Limelight AprilTag telemetry logging.
 */
@TeleOp(name = "Simple Move TeleOp", group = "Control")
public class SimpleMoveTeleOp extends LinearOpMode {

    private MecanumDrive drive;
    private Intake intake;
    private NewOuttake outtake;
    private Spindexer spindexer;
    private Flywheel flywheel;
    private Limelighter limelight;
    private String spindexerInitError = "None";
    private double currentTargetVel = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems safely with exception handling for optional hardware
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intake = new Intake(hardwareMap);

        try {
            spindexer = new Spindexer(hardwareMap, false);
        } catch (Exception e) {
            spindexer = null;
            spindexerInitError = e.getMessage();
        }

        try {
            outtake = new NewOuttake(hardwareMap);
        } catch (Exception e) {
            outtake = null;
        }

        try {
            DcMotorEx flywheelTop = hardwareMap.get(DcMotorEx.class, "flywheelMotor1");
            DcMotorEx flywheelBot = hardwareMap.get(DcMotorEx.class, "flywheelMotor2");
            flywheel = new Flywheel(flywheelTop, flywheelBot);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            flywheel = null;
        }

        try {
            limelight = new Limelighter(hardwareMap);
        } catch (Exception e) {
            limelight = null;
        }

        telemetry.addData("Status", "Initialized. Ready to start!");
        telemetry.addData("Spindexer Init Error", spindexerInitError);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // 1. Drivetrain Control (Gamepad 1)
            double drivePower = -gamepad1.left_stick_y;
            double strafePower = gamepad1.right_stick_x;
            double turnPower = gamepad1.left_stick_x;

            // Apply deadzones
            if (Math.abs(drivePower) < 0.1) drivePower = 0.0;
            if (Math.abs(strafePower) < 0.1) strafePower = 0.0;
            if (Math.abs(turnPower) < 0.1) turnPower = 0.0;

            drive.driverRelativePower(drivePower, strafePower, turnPower);

            // 2. Intake Control (Gamepad 1 Face Buttons)
            if (gamepad1.x) {
                intake.spinIntake();
            } else if (gamepad1.y) {
                intake.reverseIntake();
            } else if (gamepad1.a) {
                intake.idleIntake();
            }

            // 3. Spindexer & Kicker Control (Press B to kick and spin 1 time)
            if (spindexer != null) {
                if (gamepad1.b) {
                    spindexer.kickAndSpin();
                }
                spindexer.update(gamepad1);
            }

            // 4. Hood Angle & Flywheel Velocity Presets (D-Pad)
            // D-Up: Far Shot (Hood 0.7, Vel 1300)
            // D-Right: Mid Shot (Hood 0.5, Vel 1000)
            // D-Down: Close Shot (Hood 0.25, Vel 780)
            // D-Left: Stop Shooter (Vel 0)
            if (gamepad1.dpad_up) {
                if (outtake != null) outtake.setHoodAngle(0.7);
                currentTargetVel = 1300.0;
            } else if (gamepad1.dpad_right) {
                if (outtake != null) outtake.setHoodAngle(0.5);
                currentTargetVel = 1000.0;
            } else if (gamepad1.dpad_down) {
                if (outtake != null) outtake.setHoodAngle(0.25);
                currentTargetVel = 780.0;
            } else if (gamepad1.dpad_left) {
                currentTargetVel = 0.0;
            }

            if (flywheel != null) {
                flywheel.setVelocity(currentTargetVel);
            }

            // 5. Limelight AprilTag Telemetry Logging
            if (limelight != null) {
                try {
                    LLStatus status = limelight.getStatus();
                    if (status != null) {
                        telemetry.addData("LL FPS", "%d", (int) status.getFps());
                    }

                    LLResult result = limelight.getLatestResult();
                    if (result != null) {
                        telemetry.addData("LL Result Valid", result.isValid());
                        telemetry.addData("LL TX, TY", "%.2f°, %.2f°", result.getTx(), result.getTy());

                        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                        if (fiducials != null && !fiducials.isEmpty()) {
                            int tagId = fiducials.get(0).getFiducialId();
                            telemetry.addData("AprilTag ID", tagId);
                        }
                    }
                } catch (Exception e) {
                    // Suppress transient exceptions during telemetry polling
                }
            }

            // General Telemetry
            telemetry.addData("Drive Power", "%.2f, %.2f, %.2f", drivePower, strafePower, turnPower);
            if (outtake != null) {
                telemetry.addData("Hood Angle", "%.2f", outtake.getHoodAngle());
            }
            telemetry.addData("Shooter Velocity Target", "%.1f ticks/sec", currentTargetVel);
            if (flywheel != null) {
                telemetry.addData("Shooter Actual Velocity", "%.1f ticks/sec", flywheel.getVelocity());
            }
            if (spindexer != null) {
                telemetry.addData("Spindexer Log", spindexer.getStatusLog());
            } else {
                telemetry.addData("Spindexer Log", "NOT INITIALIZED (Missing Config)");
                telemetry.addData("Spindexer Init Error", spindexerInitError);
            }
            telemetry.addData("B Button Pressed", gamepad1.b);
            telemetry.update();
        }

        // Cleanup on OpMode stop
        if (limelight != null) {
            try {
                limelight.stopLimelight();
            } catch (Exception ignored) {}
        }
        if (flywheel != null) {
            flywheel.setPower(0.0);
        }
    }
}
