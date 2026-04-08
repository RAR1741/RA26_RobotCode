package frc.robot.subsystems;

import frc.robot.Constants.LEDConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.AddressableLED;

// import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED led;
    private AddressableLEDBuffer buffer;
    private int length;

    private LEDBufferView allLEDs;
    
    public LEDSubsystem() {
        led = new AddressableLED(LEDConstants.k_port);
        buffer = new AddressableLEDBuffer(LEDConstants.k_length);
        length = buffer.getLength();

        allLEDs = new LEDBufferView(buffer, 0, length - 1);
    
        led.setLength(length);

        led.setData(buffer);
        led.start();
        
        //TODO: set default color
    }

    @Override
    public void periodic() {
        //led.setData(convertLEDBufferIntoSimlish(buffer));
        led.setData(buffer);
    }

    @SuppressWarnings("unused")
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

    public Command setAllSolidColor(Color color) {
        return Commands.run(() -> {
            buffer = solidColor(allLEDs, color);
        }); // i dont even know why buffer is reassigned, this makes no sense
    }

    public Command setAllBlinking(Color color, int ms){
        return Commands.run(() -> {
            buffer = blinking(allLEDs, color, ms, 0);
        });
    }

    public Command setAllOff() {
        return setAllSolidColor(Color.kBlack); // black LED don't exist (maybe), so black means OFF
    }

    public Command setError() {
        return setAllBlinking(Color.kRed, 333); // 333 ms = 1/3 of second - blinks red 3x per second
    }

    public Command setAllAllianceColor() {
        return setAllSolidColor(DriverStation.getAlliance().orElse(null) == Alliance.Red ? Color.kRed : Color.kBlue);
    }

    public Command setAllColorChase(Color color, int ms) {
        return Commands.run(() -> {
            buffer = colorChase(allLEDs, color, ms, 0);
        });
    }

    public Command setAllRainbowChase(int ms) {
        return Commands.run(() -> {
            buffer = rainbowChase(allLEDs, ms, 0);
        });
    }

    // examples of more complex patterns

    public Command setSplitRainbowChase(int ms) {
        return Commands.run(() -> {
            buffer = rainbowChase(new LEDBufferView(buffer, 0, length / 2 - 1), ms, 0);

            // flipping start and end reverses the order the pattern is applied
            buffer = rainbowChase(new LEDBufferView(buffer, length - 1, length / 2), ms, 0);
        });
    }

    public Command setPacerBlinkingPattern() {
        Color[] blueFirst = {new Color("#002D62"), new Color("#FDBB30")};
        Color[] yellowFirst = {new Color("#FDBB30"), new Color("#002D62")};
        return Commands.run(() -> {
            buffer = followPattern(new LEDBufferView(buffer, 0,              length / 4 - 1    ), blueFirst, 1000, 0);
            buffer = followPattern(new LEDBufferView(buffer, length / 4,     length / 2 - 1    ), yellowFirst, 1000, 0);
            buffer = followPattern(new LEDBufferView(buffer, length / 2,     3 * length / 4 - 1), blueFirst, 1000, 0);
            buffer = followPattern(new LEDBufferView(buffer, 3 * length / 4, length - 1        ), yellowFirst, 1000, 0);
        });
    }

    // the methods behind the madness

    private AddressableLEDBuffer solidColor(LEDBufferView view, Color color) {

        for(int i = 0; i < view.getLength(); i++) {
            view.setLED(i, color);
        }

        return view.buffer;
    }

    private AddressableLEDBuffer colorChase(LEDBufferView view, Color color, int ms, int offset) {

        for (int i = 0; i < view.getLength(); i++) {
            double t = ((i + System.currentTimeMillis()) % ms) / (double) ms;
            color = Color.lerpRGB(color, Color.kBlack, t);
            view.setLED(i, color);
        }

        return view.buffer;
    };

    private AddressableLEDBuffer rainbowChase(LEDBufferView view, int ms, int offset) {
        int length = view.getLength();
        double firstPixelHue = System.currentTimeMillis() / (double) ms;

        for (int i = 0; i < length; i++) {
          view.setHSV(i, (int) (180.0 * ((firstPixelHue + i / length) % 1.0)), 255, 64);
        }

        return view.buffer;
    };

    private AddressableLEDBuffer blinking(LEDBufferView view, Color color, int ms, int offset) {

        for (int i = 0; i < view.getLength(); i++) {
            if ((System.currentTimeMillis() + offset) % ms >= ms / 2) {
                color = Color.kBlack;
            }
            view.setLED(i, color);
        }

        return view.buffer;
    };

    private AddressableLEDBuffer followPattern(LEDBufferView view, Color[] pattern, int ms, int offset) {

        for (int i = 0; i < view.getLength(); i++) {
            view.setLED(i, pattern[(int) (((double) pattern.length) * System.currentTimeMillis() / ms)]);
        }

        return view.buffer;
    }

    // replaces AddressableLEDBufferView, so that you can access the actual buffer and indices

    public class LEDBufferView {
        public final AddressableLEDBuffer buffer;
        public final int start;
        public final int end;
        public final boolean reversed;

        public LEDBufferView(AddressableLEDBuffer buffer, int start, int end) { // inclusive of start and end
            this.buffer = buffer;
            if (end >= start) {
                this.start = start;
                this.end = end;
                this.reversed = false;
            } else {
                this.start = end;
                this.end = start;
                this.reversed = true;
            }
        }

        public void setLED(int i, Color color) {
            if (i < 0 || i >= this.getLength()) {
                return;
            }
            if (reversed) {
                buffer.setLED(end - i, color);
            } else {
                buffer.setLED(i + start, color);
            }
        }

        public void setHSV(int i, int h, int s, int v) {
            if (i < 0 || i >= this.getLength()) {
                return;
            }
            if (reversed) {
                buffer.setHSV(end - i, h, s, v);
            } else {
                buffer.setHSV(i + start, h, s, v);
            }
        }

        public int getLength() {
            return end - start + 1;
        }

        public LEDBufferView reversed() {
            return new LEDBufferView(buffer, end, start);
        }
    }
}