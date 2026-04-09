package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDMinimal extends SubsystemBase {
    @SuppressWarnings({ "deprecated", "removal" })
    private final CANdle candle = new CANdle(4, "DriveCanivore");

    private final SolidColor blueRequest = new SolidColor(0, 7).withColor(new RGBWColor(0, 0, 255));
    private StatusCode lastSetControlStatus = StatusCode.StatusCodeNotInitialized;

    public LEDMinimal() {
        candle.clearAllAnimations();
        lastSetControlStatus = candle.setControl(blueRequest);
        SmartDashboard.putString("LEDMinimal/CANdleSetControl", lastSetControlStatus.toString());
    }

    @Override
    public void periodic() {
        lastSetControlStatus = candle.setControl(blueRequest);
        SmartDashboard.putString("LEDMinimal/CANdleSetControl", lastSetControlStatus.toString());
        SmartDashboard.putBoolean("LEDMinimal/CANdleSetControlOK", lastSetControlStatus.isOK());
    }
}