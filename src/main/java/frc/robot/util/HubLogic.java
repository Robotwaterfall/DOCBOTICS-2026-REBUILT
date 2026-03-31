import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class HubLogic {

  public enum HubState {
    ACTIVE,
    INACTIVE,
    WARNING
  }

  public HubState getHubState() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) {
      return HubState.INACTIVE;
    }

    if (DriverStation.isAutonomousEnabled()) {
      return HubState.ACTIVE;
    }

    if (!DriverStation.isTeleopEnabled()) {
      return HubState.INACTIVE;
    }

    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();

    if (gameData.isEmpty()) {
      return HubState.ACTIVE;
    }

    boolean redInactiveFirst = switch (gameData.charAt(0)) {
      case 'R' -> true;
      case 'B' -> false;
      default -> true;
    };

    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    return getShiftState(matchTime, shift1Active);
  }

  private HubState getShiftState(double time, boolean shift1Active) {

    if (time > 130) return HubState.ACTIVE;

    if (time > 105) return evaluate(time, 105, shift1Active);
    if (time > 80)  return evaluate(time, 80, !shift1Active);
    if (time > 55)  return evaluate(time, 55, shift1Active);
    if (time > 30)  return evaluate(time, 30, !shift1Active);

    return HubState.ACTIVE;
  }

  private HubState evaluate(double time, double threshold, boolean active) {
    // 5-second warning window before shift change
    if (time <= threshold + 5 && time > threshold) {
      return HubState.WARNING;
    }

    return active ? HubState.ACTIVE : HubState.INACTIVE;
  }
}