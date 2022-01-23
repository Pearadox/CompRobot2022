package frc.lib.drivers;

import com.revrobotics.CANSparkMax;

public class PearadoxSparkMax extends CANSparkMax {
  public PearadoxSparkMax(
      int deviceNumber, MotorType m, IdleMode i_mode, int limit, boolean restore) {
    super(deviceNumber, m);
    this.enableVoltageCompensation(12.0);
    this.setSmartCurrentLimit(limit);
    this.setIdleMode(i_mode);
    if (restore) {
      this.restoreFactoryDefaults();
    }
  }

  public PearadoxSparkMax(int deviceNumber) {
    super(deviceNumber, MotorType.kBrushless);
    this.enableVoltageCompensation(12.0);
    this.restoreFactoryDefaults();

    // Assumes worse case of neo550
    // 20 amps would be safest, but 30 could be OK. 40 will kill the motor in 30 seconds
    this.setSmartCurrentLimit(30);
    this.setIdleMode(IdleMode.kBrake);
  }
}
