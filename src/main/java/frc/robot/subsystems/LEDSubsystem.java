package frc.robot.subsystems;

import frc.robot.Constants.LEDConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Function;

public class LEDSubsystem extends SubsystemBase {
    private AddressableLEDSim led; //CHANGE BACK TO AddressableLED
    private AddressableLEDBuffer buffer;
    private int length;

    @SuppressWarnings("unused")
    private LEDModeArgs allLEDs;
    
    public LEDSubsystem() {
        led = new AddressableLEDSim();
        //led = new AddressableLED();
        buffer = new AddressableLEDBuffer(LEDConstants.k_length);
        length = buffer.getLength();

        allLEDs = new LEDModeArgs(0, length, buffer);
    
        led.setLength(length);

        led.setData(convertLEDBufferIntoSimlish(buffer));
        //led.setData(buffer);
        //led.start();
        
        // set default commando to lighto solido coloro basedo yoko ono allianceo
        // How are we supposed to know which alliance we are on when the getAlliance() in RobotContainer is private??
    }

    private static byte[] convertLEDBufferIntoSimlish(AddressableLEDBuffer buffer) {
        int length = buffer.getLength();
        byte[] data = new byte[length * 3];
        
        for (int i = 0; i < length; i++) {
            data[i * 3]     = (byte) buffer.getRed(i);
            data[i * 3 + 1] = (byte) buffer.getGreen(i);
            data[i * 3 + 2] = (byte) buffer.getBlue(i);
        }
        
        return data;
    }

    public Command setAllLEDsSolidColor(Color color) {
        return Commands.run(() -> setAllLEDsSolidColorMethod(color));
    }

    private AddressableLEDBuffer setAllLEDsSolidColorMethod(Color color) {
        for(int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
        return buffer;
    }

    public Command setAllLEDsBlinking(Color color, long ms){
        return Commands.run(() -> blinking.apply(new LEDModeArgs(0, length, buffer, color, ms)));
    }

    public Command setAllLEDsOff() {
        return setAllLEDsSolidColor(Color.kBlack); // black LED don't exist (maybe), so black means OFF
    }

    public Command setErrorLED() {
        return setAllLEDsBlinking(Color.kRed, 333); // 333 ms = 1/3 of second - blinks red 3x per second
    }

    public Command setAllLEDsAllianceColor() {
        return setAllLEDsSolidColor(RobotContainer.retrieveAlliance() == Alliance.Red ? Color.kRed : Color.kBlue);
    }

    // public Command setAllLEDsMode(Function<LEDModeArgs, AddressableLEDBuffer> mode) {
    //     return Commands.run(() -> mode.apply(allLEDs));
    // }

    public Command setAllLEDsColorChase(Color color, long ms) {
        return Commands.run(() -> colorChase.apply(new LEDModeArgs(0, length, buffer, color, ms)));
    }

    public Command setAllLEDsRainbowChase(long ms) {
        return Commands.run(() -> rainbowChase.apply(new LEDModeArgs(0, length, buffer, ms)));
    }

    public Function<LEDModeArgs, AddressableLEDBuffer> colorChase = (LEDModeArgs args) -> {
        for (int i = args.start; i < args.start + args.length; i++) {
            buffer.setLED(i, Color.lerpRGB(args.color, Color.kBlack, ((i + System.currentTimeMillis()) % args.ms) / (double) args.ms));
        }
        return args.buffer;
    };

    public Function<LEDModeArgs, AddressableLEDBuffer> rainbowChase = (LEDModeArgs args) -> {
        int firstPixelHue = (int) ((System.currentTimeMillis() / 1000.0 * args.ms) % 180);
        for (int i = args.start; i < (args.start + args.length); i++) {
          buffer.setHSV(i, firstPixelHue, 255, 64);
        }
        return args.buffer;
    };

    public Function<LEDModeArgs, AddressableLEDBuffer> blinking = (LEDModeArgs args) -> {
        for (int i = args.start; i < args.start + args.length; i++) {
            args.buffer.setLED(i, (System.currentTimeMillis() % (args.ms) < args.ms / 2.0) ? args.color : Color.kBlack);
        }
        return args.buffer;
    };

    private class LEDModeArgs { // all of the cursedness of last year's code was offloaded here
        public int start;
        public int length;
        public AddressableLEDBuffer buffer;
        public Color color;
        public long ms;

        // something JavaScript could do in a couple characters btw 
        public LEDModeArgs(int start, int length, AddressableLEDBuffer buffer) {
            this.start = start;
            this.length = length;
            this.buffer = buffer;
        }

        @SuppressWarnings("unused")
        public LEDModeArgs(int start, int length, AddressableLEDBuffer buffer, Color color) {
            this.start = start;
            this.length = length;
            this.buffer = buffer; // istg JAAAAAVAAAAAAAA AAAAAAAAAAAAAAAAAAAAAAAA WHY CANT IT DO ANYTHING
            this.color = color;   // why are we still here? just to suffer?
        }

        public LEDModeArgs(int start, int length, AddressableLEDBuffer buffer, long ms) {
            this.start = start;
            this.length = length;
            this.buffer = buffer;
            this.ms = ms;
        }

        public LEDModeArgs(int start, int length, AddressableLEDBuffer buffer, Color color, long ms) {
            this.start = start;
            this.length = length;
            this.buffer = buffer;
            this.color = color;
            this.ms = ms;
        }
    }
}
