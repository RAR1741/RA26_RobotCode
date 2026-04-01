package frc.robot.subsystems;

import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDSubsystem extends SubsystemBase{
    private AddressableLED led;
    private AddressableLEDBuffer buffer;

    enum MODES { 
        SOLID,
        BLINK
    };
    
    public LEDSubsystem() {
        led = new AddressableLED(LEDConstants.k_port);
        buffer = new AddressableLEDBuffer(LEDConstants.k_length);
    
        led.setLength(buffer.getLength());

        led.setData(buffer);
        led.start();
        
        //set default commando to lighto solido coloro basedo yoko ono allianceo
    }

    public Command setSolidColorCommand(Color color) {
        return Commands.run(() -> setSolidColor(color));
    }

    public void setSolidColor(Color color) {
        for(int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
    }
}
