package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.hardwares.RTPTorctex;

import java.util.Arrays;

public class Spindexer {
    //include spindexer servo, light, booktkicker, and color sensor
    //note:deprecate Spindexer.java once done

    // Static constants
    private static final int TIME_TO_DETECT = 250; //millis TODO: Increase delay and test
    private static final int BOOTKICKER_DELAY = 500; //millis
    public static String[] motif = new String[]{"G", "P", "P"};
    private static boolean isInitalized;

    public String x;
    public String posState;

    // Hardware
    private final RTPTorctex sorter;
    private final Servo bootkicker;
    private final ColorSensor colorSensor;

    // Constants
    private final int[] outTakePositions = {190, 310, 70}; //index 0 is the degree to send slot 1 to intake, etc
    private final int[] intakePositions = {10, 130, 250}; //index 0 is the degree to send slot 1 to outtake, etc
    // Bot Variables
    private String[] inventory = {"E", "E", "E"}; //E = empty, P = purple, G = green

    // State Variables
    private boolean canSpin;
    private boolean bootkickerCalled;
    private SpindexerMode spindexerMode;
    private SorterState sorterState;
    private KickerState kickerState;
    private ShootSequenceState shootSequenceState;
    private int currentChamber;
    private int teleopMotif;
    private boolean colorFound;
    private double colorStartTime;
    private boolean sorterAtTarget; // Track if sorter has reached target for 100ms delay
    private final ElapsedTime bootKickerTimer;
    private final ElapsedTime colorSensorTimer;
    private final ElapsedTime spindexerTimer;

    private int currentTargetMotifNum = 0;
    public String flagActive;

    private int r, g, b, opacity;


    //https://gm0.org/en/latest/docs/software/concepts/finite-state-machines.html
    //replace the ifs with a switch case
    public Spindexer(HardwareMap hardwareMap, boolean auto) {
        //Spindexer Servo
        CRServo crServo = hardwareMap.get(CRServo.class, "Spindexer Servo");
        AnalogInput analogEncoder = hardwareMap.get(AnalogInput.class, "Spindexer Encoder");
        sorter = new RTPTorctex(crServo, analogEncoder);
        RevColorSensorV3 revColorSensor = null;
        try {
            revColorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        } catch (Exception ignored) {}
        colorSensor = revColorSensor != null ? new ColorSensor(revColorSensor) : null;

        bootkicker = hardwareMap.get(Servo.class, "bootkicker");
        bootkicker.setDirection(Servo.Direction.REVERSE);

        shootSequenceState = ShootSequenceState.Ready;
        if (auto) {
            inventory = new String[] {"P", "G", "P"}; //Todo: physically label chambers 1, 2, 3
            spindexerMode = SpindexerMode.Outtake;
        }
        else {
            spindexerMode = SpindexerMode.Intake;
        }

        bootKickerTimer = new ElapsedTime();
        colorSensorTimer = new ElapsedTime();
        spindexerTimer = new ElapsedTime();
        isInitalized = false;
    }

    //https://github.com/NgDodo/FTC12209new/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystems/Sorter.java#L17

    //**
    // This update class is for auton
    // *//

    public void update() {
        if (!isInitalized) {
            bootkicker.setPosition(0);
            canSpin = true;

            spindexerMode = SpindexerMode.AutonWait;

            kickerState = KickerState.Ready;
            sorterState = SorterState.Ready;

            isInitalized = true;
//            currentChamber = 0;
            return;
        }

        //only let the Torctex PID update if the kicker is down. May have to forcibly set power in a else {.setPower(0);}
        if (kickerState.equals(KickerState.Ready)) {
            sorter.update();
        }
//        else {
//            sorter.setPower(0);
//        }

        if (spindexerMode.equals(SpindexerMode.AutonWait)) return;

        if (spindexerMode.equals(SpindexerMode.Outtake)) {
            AutoShootingSequenceAuton(); // may have to create a flag in !
        }
        else if (spindexerMode.equals(SpindexerMode.Intake) ) { //maybe check if flywheel is currently within target deadzone?
            if (colorSensor != null) colorSensor.update();
        }

        bootkickerFSM();
        handleIndexingMode();
        sorterFSM();

    }


    //use this for actions in auton, call update() at the start of the loop and then call this when
    //you want to
    public void changeSpindexerMode(SpindexerMode newMode) {
        this.spindexerMode = newMode;
    }

    public void kickAndSpin() {
        if (kickerState.equals(KickerState.Ready) && sorterState.equals(SorterState.Ready)) {
            kickerState = KickerState.SendUp;
            currentChamber = (currentChamber + 1) % 3;
            sorter.setTargetRotation(intakePositions[currentChamber]);
            sorterState = SorterState.Spinning;
            spindexerTimer.reset();
        }
    }

    public String getStatusLog() {
        return "Kicker: " + (kickerState != null ? kickerState.name() : "null") +
               " | Sorter: " + (sorterState != null ? sorterState.name() : "null") +
               " | Chamber: " + currentChamber;
    }

    /**
     * Overloaded `update()` for teleop use. Note the addition of gamepad as input
     */
    public void update(Gamepad gamepad2) {
        if (!isInitalized) { //ALWAYS BRING BOOTKICKER DOWN AFTER A RUN ALWAYS!!!!! NEVER LET IT ALIGN ON A WALL
            bootkicker.setPosition(0);
            canSpin = true;

            spindexerMode = SpindexerMode.Intake;
            sorter.setTargetRotation(intakePositions[0]);

            //FSM states initalization
            kickerState = KickerState.Ready;
            sorterState = SorterState.Ready;

            currentChamber = 0;
            isInitalized = true;
            return;
        }

        if (kickerState.equals(KickerState.Ready)) {
            sorter.update();
        }
        else {
            sorter.setPower(0);
        }

        if (gamepad2.aWasReleased()) { // choose motif from button presses, have this add to telem, prob deprecate this in final
            switch (teleopMotif) {
                case 0:
                    motif = new String[]{"G", "P", "P"};
                    teleopMotif++;
                    break;
                case 1:
                    motif = new String[]{"P", "G", "P"};
                    teleopMotif++;
                    break;
                case 2:
                    motif = new String[]{"P", "P", "G"};
                    teleopMotif = 0;
                    break;
            }
        }

        if (gamepad2.xWasReleased()) {
            setSpindexerMode(SpindexerMode.Outtake);
        }

        if (gamepad2.backWasReleased()) {
            setSpindexerMode(SpindexerMode.Intake);
        }

        //check in Intake mode, the spindexer is not changing target or moving, kicker isnt doing anything,
//        if (gamepad2.dpadUpWasReleased() && kickerState.equals(KickerState.Ready)
//                && sorterState.equals(SorterState.Ready) && spindexerMode.equals(SpindexerMode.Intake)) {
//            //call fsm for servo
//            kickerState = KickerState.SendUp;
//        }

        switch (spindexerMode) {
            case Intake:
                gamepad2.setLedColor(0, 1, 0, 100000);
                break;
            case Outtake:
                gamepad2.setLedColor(1, 0, 0, 100000);
                break;
        }

        bootkickerFSM();
        handleIndexingMode();
        sorterFSM();
        //shooting modes
    }

    public void bootkickerFSM() {
        switch (kickerState) {
            case Ready:
                bootkickerCalled = false;
                bootKickerTimer.reset();
                break;
            case SendUp:
                if (bootKickerTimer.milliseconds() >= 200) {
                    bootkickerCalled = true;
                    bootkicker.setPosition(0.4);
                    kickerState = KickerState.SendDown;
                    bootKickerTimer.reset();
                }
                break;
            case SendDown:
                if (bootKickerTimer.milliseconds() >= BOOTKICKER_DELAY) {
                    bootkicker.setPosition(0);
                    kickerState = KickerState.WaitTillDown;
                    bootKickerTimer.reset();
                }
                break;
            case WaitTillDown:
                if (bootKickerTimer.milliseconds() >= BOOTKICKER_DELAY) {
                    kickerState = KickerState.Ready;
                }
                break;
        }
    }




    //Angle Rotations for intake:
    //Slot 1: 60?
    //slot 2; 120?
    //slot 3: 340?

    //Angle Rotations for Outtake/under flywheel
    //slot 1: 200 or 220??
    //slot 2:
    //slot 3: 30
    //https://docs.ftclib.org/ftclib/features/util#timing-functions replace timers with this

    public void sorterFSM() {
        switch (sorterState) {
            case Ready:
                sorterAtTarget = false; // Reset flag when returning to Ready
                break;
            case Spinning:
                // Check if sorter has reached target for at least 100 ms
                if (sorter.isAtTarget()) {
                    if (!sorterAtTarget) {
                        // First time reaching target - reset timer
                        sorterAtTarget = true;
                        spindexerTimer.reset();
                    }
                    // Check if we've been at target for 100ms
                    if (spindexerTimer.milliseconds() >= 200) {
                        sorterState = SorterState.Ready;
                    }
                } else {
                    // Lost target - reset the flag
                    sorterAtTarget = false;
                }
                break;
            case SpinToEmptyChamber: //only use for intake
                for (int i = 0; i < 3; i++) {
                    if (inventory[i].equals("E")) {
                        currentChamber = i;
                        sorter.setTargetRotation(intakePositions[i]);
                        sorterState = SorterState.Spinning;
                        spindexerTimer.reset();
                        break;
                    }
                }
                break;
            case SpinToOuttakeTargetingMotif:
                // doesnt account if nothing in inventory matches
                int targetPos = getTargetChamberForMotif();
                sorter.setTargetRotation(outTakePositions[targetPos]);
                inventory[targetPos] = "E";
                sorterState = SorterState.Spinning;
                spindexerTimer.reset();
                break;
        }
    }

    public int getTargetChamberForMotif() {
        for (int i = 0; i < 3; i++) {
            if (inventory[i].equals(motif[currentTargetMotifNum])) {
                // REMOVED: inventory[targetPos] = "E";
                return i;
            }
        }
        for (int j = 0; j < 3; j++) {
            if (!inventory[j].equals("E")) {
                return j;
            }
        }
        return 0;
    }

    public boolean canSpin() {
        return (!bootkickerCalled && sorterState.equals(SorterState.Ready));
    }

    public void setSpindexerMode(SpindexerMode mode) {
        this.spindexerMode = mode;
        if (mode == SpindexerMode.Outtake) {
            // Force the state machine to start
            this.currentTargetMotifNum = 0;
            this.shootSequenceState = ShootSequenceState.ChangeChamber; // Start immediately
            this.sorterState = SorterState.Ready; // Interrupt any current spin
        } else {
            this.sorterState = SorterState.SpinToEmptyChamber;
        }
    }

    public void handleIndexingMode() {
        if (!canSpin()) return;

        if (spindexerMode.equals(SpindexerMode.Outtake)) {
            // Just run the sequence; the mode-switch handles the start state
            AutoShootingSequenceAuton();
        }
        else if (spindexerMode.equals(SpindexerMode.Intake) && !isFull()) {
            if (colorSensor != null) colorSensor.update();
        }
    }

    public void AutoShootingSequenceAuton() {
        // Only exit if we are Ready (not in the middle of a shot) AND empty
        if (shootSequenceState == ShootSequenceState.Ready && isEmpty()) {
            spindexerMode = SpindexerMode.Intake;
            currentTargetMotifNum = 0;
            sorterState = SorterState.SpinToEmptyChamber;
            return;
        }

        switch (shootSequenceState) {
            case Ready:
                if (spindexerMode == SpindexerMode.Outtake && !isEmpty()) {
                    shootSequenceState = ShootSequenceState.ChangeChamber;
                }
                break;

            case ChangeChamber:
                if (sorterState == SorterState.Ready && kickerState == KickerState.Ready) {
                    // Determine target
                    int targetPos = getTargetChamberForMotif();
                    sorter.setTargetRotation(outTakePositions[targetPos]);

                    // Store which chamber we are about to shoot so we can empty it later
                    currentChamber = targetPos;

                    sorterState = SorterState.Spinning; // Set to spinning so we wait for arrival
                    shootSequenceState = ShootSequenceState.WaitForSpindexer;

                }
                break;
            case WaitForSpindexer:
                if (spindexerTimer.milliseconds() >= 500 && sorterState == SorterState.Ready) {
                    shootSequenceState = ShootSequenceState.KickArtifact;
                }
                break;
            case KickArtifact:
                if (sorterState == SorterState.Ready) {
                    kickerState = KickerState.SendUp;
                    // NOW we mark it as empty, after the sorter arrived and kicker started
                    inventory[currentChamber] = "E";
                    currentTargetMotifNum = (currentTargetMotifNum + 1) % 3;

                    shootSequenceState = ShootSequenceState.WaitForKicker;
                }
                break;

            case WaitForKicker:
                if (kickerState == KickerState.Ready) {
                    shootSequenceState = ShootSequenceState.Ready;
                }
                break;
        }
    }

    private boolean readyToShoot() {
        if (kickerState.equals(KickerState.Ready) && sorterState.equals(SorterState.Ready)) {
            return true;

        }
        return false;
    }

    public class ColorSensor {
        private final RevColorSensorV3 revColorSensor;
        public ColorSensor(RevColorSensorV3 revColorSensor) {
            this.revColorSensor = revColorSensor;
        }

        public void update() {
//         NormalizedRGBA colors = colorSensor.getNormalizedColors();
//         RGB = new double[] {colors.red, colors.green, colors.blue};
            String colorDetected = detectArtifactColor();

            if (colorDetected.equals("E")) {
                colorFound = false;
                return;
            }

            if (!colorFound) {
                colorFound = true;
                colorSensorTimer.reset();
                colorStartTime = colorSensorTimer.milliseconds();
            }

            if (colorSensorTimer.milliseconds() - colorStartTime >= TIME_TO_DETECT) {
                if (inventory[currentChamber].equals("E")) {
                    inventory[currentChamber] = colorDetected;
                    sorterState = SorterState.SpinToEmptyChamber; //if full/last chamber, send to outtake pos 1 beforehand to prevent ball from falling out ?
                    flagActive = "flaged";
                }
                colorStartTime = 0;
                colorFound = true;
                //deactivate colorSensor
            }
            else {
                flagActive = "Not flaged";
            }
        }
        //TODO: RETUNE COLOR SENSOR
        //TODO: Optional get feedback from the GoBuilda light when updating/detecting a artifact
        public String detectArtifactColor() {
            r = revColorSensor.red(); //test if you dont need remove g> r and b > r and see if it still works
            g = revColorSensor.green();
            b = revColorSensor.blue();
            opacity = getARGB();
            if (opacity <= 140000000) { // higher opacity means no object detected
                return "E";
            } else if (g > r && g > b && g > 50 && g < 600) { //sometimes g or b will go to some absurd value in the hundreds
                return "G";
            } else if (b > r && b > g && b > 50 && b < 600) { // for purple check
                return "P";
            } else return "E";
        }

        public int getARGB() {
            return revColorSensor.argb();
        }

        public String log() {

            return "Red: " + r
                    + "\nGreen: " + g
                    + "\nBlue: " + b
                    + "\nARGB: " + getARGB()
                    + "\nArtifact Color: " + detectArtifactColor() +
                    "Is able to spin: " + canSpin;
        }
    }

    public void setMotif(int motifTagNum) { //may not be needed
        if (motifTagNum == 21) {
            motif = new String[]{"G", "P", "P"};
        } else if (motifTagNum == 22) {
            motif = new String[]{"P", "G", "P"};
        } else if (motifTagNum == 23) {
            motif = new String[]{"P", "P", "G"};
        }
    }

    public String[] getCurrentMotif() {
        return motif;
    }



    public String printInventory () {
        return "Chamber 1: " + inventory[0] + "\n Chamber 2: " +  inventory[1] + "\n Chamber 3: " + inventory[2];
    }

    public boolean isFull() {
        for (int i = 0; i < 3; i++) {
            if (inventory[i].equals("E")) {
                return false;
            }
        }
        return true;
    }

    private boolean isEmpty() {
        for (int i = 0; i < 3; i++) {
            if (inventory[i].equals("P") || inventory[i].equals("G")) {
                return false;
            }
        }
        return true;
    }
    public boolean checkInventory(int targetColor) {
        for (int j = 0; j < 3; j++) {
            if (inventory[j].equals(motif[targetColor])) {
                sorter.changeTargetRotation(outTakePositions[j]);
                return true;
            }
        }
        return false;
    }

//    public void feedFromIntake () {

    private enum SpindexerMode {
        Intake,
        Outtake,
        AutonWait
    }


    private enum SorterState {
        Ready,
        SpinToEmptyChamber,
        Spinning,
        SpinToOuttakeTargetingMotif
    }

    private enum KickerState {
        Ready,
        SendUp,
        SendDown,
        WaitTillDown
    }

    private enum ShootSequenceState {
        Ready,
        ChangeChamber,
        WaitForKicker,
        WaitForSpindexer,
        KickArtifact
    }

    @TeleOp(name = "Test Spindexer", group = "Testing")
    public static class ColorSensorTest extends LinearOpMode {

        @Override
        public void runOpMode() {

            waitForStart();
            Spindexer spindexer = new Spindexer(this.hardwareMap, false);


            while (!isStopRequested()) {
                spindexer.update(gamepad1);

//                if (gamepad1.dpadDownWasReleased()) {
//                    spindexer.resetServo();
//                }
//                if (gamepad1.dpadUpWasReleased()){
//                    spindexer.kick();
//                }

                telemetry.addLine(spindexer.colorSensor.log());
                telemetry.addData("x: ", spindexer.x);
                telemetry.addData("Pos State: ", spindexer.posState);
                telemetry.addData("Spindexer Power", spindexer.sorter.getPower());
                telemetry.addLine(spindexer.flagActive);
                telemetry.addLine(spindexer.printInventory());
                telemetry.addData("Inventory", Arrays.toString(spindexer.inventory));
                telemetry.addData("Shoot State", spindexer.shootSequenceState);
                telemetry.update();
            }
        }
    }
}
