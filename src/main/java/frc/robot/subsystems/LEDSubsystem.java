package frc.robot.subsystems;

import frc.robot.Constants.LEDConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Function;

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED led;
    private AddressableLEDBuffer buffer;
    private int length;

    private LEDModeArgs allLEDs;

    enum MODES { 
        SOLID,
        BLINK
    };
    
    public LEDSubsystem() {
        led = new AddressableLED(LEDConstants.k_port);
        buffer = new AddressableLEDBuffer(LEDConstants.k_length);
        length = buffer.getLength();

        allLEDs = new LEDModeArgs(0, length, buffer);
    
        led.setLength(length);

        led.setData(buffer);
        led.start();
        
        // set default commando to lighto solido coloro basedo yoko ono allianceo
        // How are we supposed to know which alliance we are on when the getAlliance() in RobotContainer is private??
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

    public Command setAllLEDsOff() {
        return setAllLEDsSolidColor(Color.kBlack);
    }

    public Command setAllLEDsMode(Function<LEDModeArgs, AddressableLEDBuffer> mode) {
        return Commands.run(() -> mode.apply(allLEDs));
    }

    public AddressableLEDBuffer rainbowChase(LEDModeArgs args) {
        // do the thing
        return args.buffer;
    }

    private class LEDModeArgs {
        public int start;
        public int length;
        public AddressableLEDBuffer buffer;

        public LEDModeArgs(int start, int length, AddressableLEDBuffer buffer) {
            this.start = start;
            this.length = length;
            this.buffer = buffer;
        }
    }
}
