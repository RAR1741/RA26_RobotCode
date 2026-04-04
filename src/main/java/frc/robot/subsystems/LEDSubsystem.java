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

    private LEDBufferView allLEDs;
    
    public LEDSubsystem() {
        led = new AddressableLEDSim();
        //led = new AddressableLED();
        buffer = new AddressableLEDBuffer(LEDConstants.k_length);
        length = buffer.getLength();

        allLEDs = new LEDBufferView(buffer, 0, length - 1);
    
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
        return Commands.run(() -> blinking.apply(new LEDModeArgs(color, ms)));
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
        return Commands.run(() -> colorChase.apply(new LEDModeArgs(color, ms)));
    }

    public Command setAllLEDsRainbowChase(long ms) {
        return Commands.run(() -> rainbowChase.apply(new LEDModeArgs(ms)));
    }

    public Function<LEDModeArgs, AddressableLEDBuffer> colorChase = (LEDModeArgs args) -> {

        for (int i = 0; i < args.view.getLength(); i++) {
            double t = ((i + System.currentTimeMillis()) % args.ms) / (double) args.ms;
            Color color = Color.lerpRGB(args.color, Color.kBlack, t);
            args.view.setLED(i, color);
        }

        return args.view.buffer;
    };

    public Function<LEDModeArgs, AddressableLEDBuffer> rainbowChase = (LEDModeArgs args) -> {
        int firstPixelHue = (int) ((System.currentTimeMillis() / 1000.0 * args.ms) % 180);

        for (int i = 0; i < args.view.getLength(); i++) {
          args.view.setHSV(i, firstPixelHue, 255, 64);
        }

        return args.view.buffer;
    };

    public Function<LEDModeArgs, AddressableLEDBuffer> blinking = (LEDModeArgs args) -> {
        for (int i = 0; i < args.view.getLength(); i++) {
            Color color = (System.currentTimeMillis() % args.ms < args.ms / 2) ? args.color : Color.kBlack;
            args.view.setLED(i, color);
        }
        return args.view.buffer;
    };

    private class LEDBufferView {
        public AddressableLEDBuffer buffer;
        public int start;
        public int end;
        public boolean reversed;

        public LEDBufferView(AddressableLEDBuffer buffer, int start, int end) {
            if (end == start) {
                throw new Exception("end cannot be same as start");
            } else if (end < start) {
                int t = end;
                end = start;
                start = t;
                this.reversed = true;
            } else {
                this.reversed = false;
            }
            this.buffer = buffer;
            this.start = start;
            this.end = end;
            this.reversed = false;
        }

        public void setLED(int i, Color color) {
            buffer.setLED(i + start, color); // make sure not off by one
        }

        public int getLength() {
            return Math.abs(end - start) + 1;
        }
    }

    private class LEDModeArgs { // all of the cursedness of last year's code was offloaded here
        public LEDBufferView view;
        public Color color;
        public int ms;
        public int offset;

        // something JavaScript could do in a couple characters btw 
        public LEDModeArgs() {
            this.view = LEDSubsystem.allLEDs;
        }

        public LEDModeArgs(Color color) {
            this.view = LEDSubsystem.allLEDs;
            this.color = color;
        }

        public LEDModeArgs(int ms) {
            this.view = LEDSubsystem.allLEDs;
            this.ms = ms;
            this.offset = 0;
        }

        public LEDModeArgs(Color color, int ms) {
            this.view = LEDSubsystem.allLEDs;
            this.color = color;
            this.ms = ms;
            this.offset = 0;
        }

        public LEDModeArgs(int ms, int offset) {
            this.view = LEDSubsystem.allLEDs;
            this.ms = ms;
            this.offset = offset;
        }

        public LEDModeArgs(Color color, int ms, int offset) {
            this.view = LEDSubsystem.allLEDs;
            this.color = color;
            this.ms = ms;
            this.offset = offset;
        }

        public LEDModeArgs(AddressableLEDBufferView view, Color color, int ms, int offset) {
            this.view = view;
            this.color = color;
            this.ms = ms;
            this.offset = offset;
        }
    }
}
