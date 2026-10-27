package org.firstinspires.ftc.teamcode.subsystems.hardwares;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

//singultech
@Config
public class RTPTorctex {
    // ...existing code...
    // FTC Dashboard configuration for PID tuning
    public static double DASHBOARD_kP = 0.032;
    public static double DASHBOARD_kI = 0.0000;
    public static double DASHBOARD_kD = 0.00125;
    public static double DASHBOARD_MAX_INTEGRAL_SUM = 100.0;
    public static double DASHBOARD_MAX_POWER = 0.9;

    // Encoder for servo position feedback
    private final AnalogInput servoEncoder;
    // Continuous rotation servo
    private final CRServo servo;
    // Run-to-position mode flag
    private boolean rtp;
    // Current power applied to servo
    private double power;
    // Maximum allowed power
    private double maxPower;
    // Direction of servo movement
    private Direction direction;
    // Last measured angle
    private double previousAngle;
    // Accumulated rotation in degrees
    private double totalRotation;
    // Target rotation in degrees

    private static double targetRotation;

    public static double TRACK_CURRENT_ANGLE;

    // PID controller coefficients and state
    private double kP;
    private double kI;
    private double kD;
    private double integralSum;
    private double lastError;
    private double maxIntegralSum;
    private ElapsedTime pidTimer;

    // Initialization and debug fields
    public double STARTPOS;
    public int ntry = 0;
    public int cliffs = 0;
    public double homeAngle;

    private boolean atPos;

    // Direction enum for servo
    public enum Direction {
        FORWARD,
        REVERSE
    }

    // region constructors

    // Basic constructor, defaults to FORWARD direction
    public RTPTorctex(CRServo servo, AnalogInput encoder) {
        rtp = true;
        this.servo = servo;
        servoEncoder = encoder;
        direction = Direction.FORWARD;
        initialize();
    }

    // Constructor with explicit direction
    public RTPTorctex(CRServo servo, AnalogInput encoder, Direction direction) {
        this(servo, encoder);
        this.direction = direction;
        initialize();
    }

    // Initialization logic for servo and encoder
    private void initialize() {
        servo.setPower(0);
        try {
            Thread.sleep(50);
        } catch (InterruptedException ignored) {
        }

        // Try to get a valid starting position
        do {
            STARTPOS = getCurrentAngle();
            if (Math.abs(STARTPOS) > 1) {
                previousAngle = getCurrentAngle();
            } else {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException ignored) {
                }
            }
            ntry++;
        } while (Math.abs(previousAngle) < 0.2 && (ntry < 50));

        totalRotation = 0;
        homeAngle = previousAngle;

        // Default PID coefficients - load from dashboard config
        kP = DASHBOARD_kP;
        kI = DASHBOARD_kI;
        kD = DASHBOARD_kD;
        integralSum = 0.0;
        lastError = 0.0;
        maxIntegralSum = DASHBOARD_MAX_INTEGRAL_SUM;
//
//        kP = 0;
//        kI = 0;
//        kD = 0;
//        maxIntegralSum = 0;
        pidTimer = new ElapsedTime();
        pidTimer.reset();

        maxPower = DASHBOARD_MAX_POWER;
        cliffs = 0;
    }
    // endregion

    // Set servo direction
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    // Set power to servo, respecting direction and maxPower
    public void setPower(double power) {
        this.power = Math.max(-maxPower, Math.min(maxPower, power));
        servo.setPower(this.power * (direction == Direction.REVERSE ? -1 : 1));
    }

    // Get current power
    public double getPower() {
        return power;
    }

    // Set maximum allowed power
    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    // Get maximum allowed power
    public double getMaxPower() {
        return maxPower;
    }

    // Enable or disable run-to-position mode
    public void setRtp(boolean rtp) {
        this.rtp = rtp;
        if (rtp) {
            resetPID();
        }
    }

    // Get run-to-position mode state
    public boolean getRtp() {
        return rtp;
    }

    // Set PID P coefficient
    public void setKP(double kP) {
        this.kP = kP;
    }

    // Set PID I coefficient and reset integral
    public void setKI(double kI) {
        this.kI = kI;
        resetIntegral();
    }

    // Set PID D coefficient
    public void setKD(double kD) {
        this.kD = kD;
    }

    // Set all PID coefficients
    public void setPidCoeffs(double kP, double kI, double kD){
        setKP(kP);
        setKI(kI);
        setKD(kD);
    }

    // Get PID P coefficient
    public double getKP() {
        return kP;
    }

    // Get PID I coefficient
    public double getKI() {
        return kI;
    }

    // Get PID D coefficient
    public double getKD() {
        return kD;
    }

    // Set only P coefficient (alias)
    public void setK(double k) {
        setKP(k);
    }

    // Get only P coefficient (alias)
    public double getK() {
        return getKP();
    }

    // Set maximum allowed integral sum
    public void setMaxIntegralSum(double maxIntegralSum) {
        this.maxIntegralSum = maxIntegralSum;
    }

    // Get maximum allowed integral sum
    public double getMaxIntegralSum() {
        return maxIntegralSum;
    }

    // Get total rotation since initialization
    public double getTotalRotation() {
        return totalRotation;
    }

    // Get current target rotation
    public double getTargetRotation() {
        return targetRotation;
    }

    // Increment target rotation by a value
    public void changeTargetRotation(double change) {
        targetRotation = change;
    }

    // Set target rotation and reset PID
    public void setTargetRotation(double target) {
        targetRotation = target;
        resetPID();
    }

    // Get current angle from encoder (in degrees)
    public double getCurrentAngle() {
        if (servoEncoder == null) return 0;
        return (servoEncoder.getVoltage() / 3.3) * (direction.equals(Direction.REVERSE) ? -360 : 360);
    }

    // Check if servo is at target (default tolerance)
    public boolean isAtTarget() {
        return isAtTarget(2);
    }

    // Check if servo is at target (custom tolerance)
    public boolean isAtTarget(double tolerance) {
        return Math.abs(targetRotation - totalRotation) < tolerance;
    }

    // Force reset total rotation and PID state
    public void forceResetTotalRotation() {
        totalRotation = 0;
        previousAngle = getCurrentAngle();
        resetPID();
    }

    // Reset PID controller state
    public void resetPID() {
        resetIntegral();
        lastError = 0;
        pidTimer.reset();
    }

    // Reset integral sum
    public void resetIntegral() {
        integralSum = 0;
    }

    public boolean isAtTargetPos () {
        return atPos;
    }

    // Sync PID coefficients from FTC Dashboard configuration
    private void syncDashboardConfig() {
        kP = DASHBOARD_kP;
        kI = DASHBOARD_kI;
        kD = DASHBOARD_kD;
        maxIntegralSum = DASHBOARD_MAX_INTEGRAL_SUM;
        maxPower = DASHBOARD_MAX_POWER;
    }

    // Main update loop: updates rotation, computes PID, applies power
    public synchronized void update() {
        // Sync latest values from FTC Dashboard
        syncDashboardConfig();

        double currentAngle = getCurrentAngle();
        TRACK_CURRENT_ANGLE = currentAngle; //for telem
        double angleDifference = currentAngle - previousAngle;

        // Handle wraparound at 0/360 degrees
        if (angleDifference > 180) {
            angleDifference -= 360;
            cliffs--;
        } else if (angleDifference < -180) {
            angleDifference += 360;
            cliffs++;
        }

        // Update total rotation with wraparound correction
        totalRotation = currentAngle + cliffs * 360;
        previousAngle = currentAngle;

        if (!rtp) return;

        double dt = pidTimer.seconds();
        pidTimer.reset();

        // Ignore unreasonable dt values
        if (dt < 0.001 || dt > 1.0) {
            return;
        }

        double error = targetRotation - totalRotation;

        // PID integral calculation with clamping
        integralSum += error * dt;
        integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));

        // Integral wind-down in deadzone
        final double INTEGRAL_DEADZONE = 2.0;
        if (Math.abs(error) < INTEGRAL_DEADZONE) {
            integralSum *= 0.95;
        }

        // PID derivative calculation
        double derivative = (error - lastError) / dt;
        lastError = error;

        // PID output calculation
        double pTerm = kP * error;
        double iTerm = kI * integralSum;
        double dTerm = kD * derivative;

        double output = pTerm + iTerm + dTerm;

        // Deadzone for output
        final double DEADZONE = .25;
        if (Math.abs(error) > DEADZONE) {

            double power = Math.min(maxPower, Math.abs(output)) * Math.signum(output);
            setPower(power);
            atPos = false;
        }
        else {
            setPower(0);
            atPos = true;
        }
    }

    // Log current state for telemetry/debug
    @SuppressLint("DefaultLocale")
    public String log() {
        return String.format(
                "Current Volts: %.3f\n" +
                        "Current Angle: %.2f\n" +
                        "Total Rotation: %.2f\n" +
                        "Target Rotation: %.2f\n" +
                        "Current Power: %.3f\n" +
                        "PID Values: P=%.3f I=%.3f D=%.4f\n" +
                        "PID Terms: Error=%.2f Integral=%.2f",
                servoEncoder.getVoltage(),
                getCurrentAngle(),
                totalRotation,
                targetRotation,
                power,
                kP, kI, kD,
                targetRotation - totalRotation,
                integralSum
        );
    }

    // TeleOp test class for manual tuning and testing
    @TeleOp(name = "Torctex PID Test", group = "Testing")
    public static class CRTorctexTest extends LinearOpMode {
        //label each slot 1 2 and 3 with tape and let 1 be the intake then determine the angles to change each slot from intake to outake
        @Override
        public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            CRServo crservo = hardwareMap.crservo.get("Spindexer Servo"); // CHANGE NAME
            AnalogInput encoder = hardwareMap.get(AnalogInput.class, "Spindexer Encoder"); // CHANGE NAME
//            GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
//            Gamepad gamepad1 = new Gamepad();
            RTPTorctex servo = new RTPTorctex(crservo, encoder);

            waitForStart();

            while (!isStopRequested()) {
//                gamepad1.copyStates();
                servo.update();

                // Manual controls for target and PID tuning
                if (gamepad1.dpadUpWasReleased()) {
                    servo.changeTargetRotation(0);
                }
                if (gamepad1.dpadDownWasReleased()) {
                    servo.changeTargetRotation(90);
                }
                if (gamepad1.dpadLeftWasReleased()) {
                    servo.changeTargetRotation(180);
                }
                if (gamepad1.dpadRightWasReleased()) {
                    servo.changeTargetRotation(340);
                }

                if (gamepad1.xWasReleased()) {
                    servo.changeTargetRotation(-180);
                }

                if (gamepad1.yWasReleased()) {
                    servo.setKP(servo.getKP() + 0.001);
                }
                if (gamepad1.aWasReleased()) {
                    servo.setKP(Math.max(0, servo.getKP() - 0.001));
                }

                if (gamepad1.rightBumperWasReleased()) {
                    servo.setKD(servo.getKD() + 0.0001);
                }
                if (gamepad1.leftBumperWasReleased()) {
                    servo.setKD(Math.max(0, servo.getKD() - 0.0001));
                }

                if (gamepad1.bWasPressed()) {
                    servo.setKP(0.015);
                    servo.setKI(0.0000);
                    servo.setKD(0.0025);
                    servo.resetPID();
                }

                telemetry.addData("Starting angle", servo.STARTPOS);
                telemetry.addData("Target rotation", targetRotation);
                telemetry.addData("Current Angle", TRACK_CURRENT_ANGLE);
                telemetry.addLine(servo.log()); // debugging
                telemetry.addData("NTRY", servo.ntry);
                telemetry.update();
            }
        }
    }
}