package org.firstinspires.ftc.teamcode.subsystems.hardwares;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Flywheel {
    private final DcMotorEx topMotor;
    private final DcMotorEx botMotor;

    public Flywheel(DcMotorEx topMotor, DcMotorEx botMotor) {
        this.topMotor = topMotor;
        this.botMotor = botMotor;

        this.topMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.botMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        topMotor.setPower(power);
        botMotor.setPower(power);
    }

    public void setVelocity(double ticksPerSec) {
        topMotor.setVelocity(ticksPerSec);
        botMotor.setVelocity(ticksPerSec);
    }

    public void setMode(DcMotor.RunMode mode) {
        topMotor.setMode(mode);
        botMotor.setMode(mode);
    }

    public double getVelocity() {
        // Returns the average velocity of both motors
        return topMotor.getVelocity();
    }
}