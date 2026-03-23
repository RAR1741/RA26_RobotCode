package frc.robot.wrappers;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class REVThroughBoreEncoder extends DutyCycleEncoder {
  public REVThroughBoreEncoder(int channel) {
    super(channel);

    // Based on logging:
    // - Initializing: 764 ± 5
    // - Ready: 973 ± 5
    setConnectedFrequencyThreshold(900);
  }

  public REVThroughBoreEncoder(int channel, double expectedFrequency) {
    super(channel);

    // Based on logging:
    // - Initializing: 764 ± 5
    // - Ready: 973 ± 5
    setConnectedFrequencyThreshold(900);

    // Set correct frequency
    setAssumedFrequency(expectedFrequency); // Hz
  }
}
