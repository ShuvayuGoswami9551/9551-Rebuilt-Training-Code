package frc.robot.subsystems.led;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class LED extends SubsystemBase {
  private final CANdle candle = new CANdle(LEDConstants.CANdleId, LEDConstants.CANBus);

  public enum LEDState {
    INITIAL,
    OFF,
    SHOOTING,
    READY_TO_SHOOT,
    TRENCH,
    INTAKING,
    INTAKE_STOWED,
    PASSING,
    CLIMBING,
    AUTO
  }

  @Getter @Setter @AutoLogOutput private LEDState goal = LEDState.INITIAL;

  public LED() {
    var config = new CANdleConfiguration();

    // Keep this simple first. If your strip color order looks wrong,
    // change this in Tuner X or set the LED config for your strip type.
    candle.getConfigurator().apply(config);

    // Default to off
    setSolid(Color.kBlack);
  }

  @Override
  public void periodic() {
    var alliance = DriverStation.getAlliance();
    Color allianceColor = Color.kBlack;
    if (alliance.isPresent()) {
      allianceColor = alliance.get() == DriverStation.Alliance.Blue ? Color.kBlue : Color.kRed;
    }

    switch (goal) {
      case INITIAL -> setSolid(allianceColor);
      case OFF -> setSolid(Color.kBlack);
      case SHOOTING -> setStrobe(Color.kGreen, LEDConstants.fastStrobeHz);
      case READY_TO_SHOOT -> setSolid(Color.kGreen);
      case PASSING -> setStrobe(Color.kPurple, LEDConstants.fastStrobeHz);
      case TRENCH -> setSolid(Color.kYellow);
      case INTAKING -> setFade(allianceColor);
      case INTAKE_STOWED -> setFade(Color.kOrange);
      case CLIMBING -> setRainbow();
      case AUTO -> setStrobe(Color.kGold, LEDConstants.autoStrobeHz);
      default -> setSolid(Color.kBlack);
    }
  }

  private void clearAnimation() {
    candle.setControl(new EmptyAnimation(0));
  }

  private void setSolid(Color color) {
    clearAnimation();
    candle.setControl(
        new SolidColor(LEDConstants.ledStartIndex, LEDConstants.ledEndIndex)
            .withColor(new RGBWColor(color)));
  }

  private void setStrobe(Color color, double hz) {
    candle.setControl(
        new StrobeAnimation(LEDConstants.ledStartIndex, LEDConstants.ledEndIndex)
            .withColor(new RGBWColor(color))
            .withFrameRate(hz));
  }

  private void setFade(Color color) {
    candle.setControl(
        new SingleFadeAnimation(LEDConstants.ledStartIndex, LEDConstants.ledEndIndex)
            .withColor(new RGBWColor(color))
            .withFrameRate(LEDConstants.waveFadeHz));
  }

  private void setRainbow() {
    candle.setControl(
        new RainbowAnimation(LEDConstants.ledStartIndex, LEDConstants.ledEndIndex)
            .withFrameRate(LEDConstants.rainbowHz));
  }
}
