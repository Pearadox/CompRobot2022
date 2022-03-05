package frc.lib.drivers;

import com.revrobotics.CANSparkMax;

public class PearadoxSparkMax extends CANSparkMax {
  public PearadoxSparkMax(
      int deviceNumber, MotorType m, IdleMode i_mode, int limit) {
    super(deviceNumber, m);
    this.restoreFactoryDefaults();
    this.enableVoltageCompensation(12.0);
    this.setSmartCurrentLimit(limit);
    this.setIdleMode(i_mode);
    burnFlash();
  }

  public static class PearadoxNeo extends PearadoxSparkMax {
    public PearadoxNeo(
      int deviceNumber, IdleMode i_mode) {
    super(deviceNumber, MotorType.kBrushless, i_mode, 40);  
    // this.restoreFactoryDefaults();
    // this.enableVoltageCompensation(12.0);

    // Assumes worse case of neo550
    // 20 amps would be safest, but 30 could be OK. 40 will kill the motor in 30 seconds
    // this.setSmartCurrentLimit(30);
    // this.setIdleMode(i_mode);
    // burnFlash();
    }
  }
}
