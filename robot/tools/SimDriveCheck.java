import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.WebSocket;
import java.time.Duration;
import java.util.Locale;
import java.util.concurrent.CompletionStage;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Scripted driver for the WPILib desktop simulation: plays the Driver Station and an Xbox
 * controller over the HAL WebSocket extension, so a sim session can be repeated exactly.
 *
 * <pre>
 *   terminal 1:  cd robot && ./gradlew simulateJava -PsimHeadless | tee sim.log
 *   terminal 2:  java tools/SimDriveCheck.java
 * </pre>
 *
 * Plain JDK 17, no dependencies. It only sends inputs and prints what phase it is in (and any
 * rumble the robot sends back); judge the result from the robot's "[PT]" status lines in sim.log.
 * With the normal sim GUI you can do the same steps by hand with a real Xbox controller.
 */
public final class SimDriveCheck {
  // Xbox axes: 0 left X, 1 left Y, 2 left trigger, 3 right trigger, 4 right X, 5 right Y.
  // Buttons (1-based in WPILib, 0-based here): 3 Y, 4 LB, 5 RB, 6 Back, 7 Start.
  static final int Y = 3, RB = 5, BACK = 6, START = 7;

  static final class Inputs {
    double leftX, leftY, rightX, trigger;
    int pov = -1;
    boolean[] buttons = new boolean[10];
    boolean enabled, autonomous;
    String station = "blue1";
  }

  static final Inputs in = new Inputs();
  static final long T0 = System.nanoTime();

  public static void main(String[] args) throws Exception {
    WebSocket ws = connect();
    Thread sender =
        new Thread(
            () -> {
              while (true) {
                try {
                  send(ws);
                  Thread.sleep(20);
                } catch (Exception e) {
                  return;
                }
              }
            });
    sender.setDaemon(true);
    sender.start();

    phase("A: teleop enabled (blue), trigger released, sticks centered -> expect OFF, robot still");
    in.enabled = true;
    hold(3.0);

    phase("B: hold the right trigger -> expect ASSIST: ENGAGED, robot tracks the Line mirror target");
    in.trigger = 1.0;
    hold(20.0);

    phase("C: trigger held + left stick forward 0.6 -> expect DRIVER_OVERRIDE, robot drives +x");
    in.leftY = -0.6;
    hold(2.0);

    phase("D: stick released, trigger still held -> expect ASSIST again");
    in.leftY = 0.0;
    hold(5.0);

    phase("E: D-pad right / down / left / up -> Cut-off, Shadow, Zone, Line mirror (2 s each)");
    for (int pov : new int[] {90, 180, 270, 0}) {
      in.pov = pov;
      hold(0.2);
      in.pov = -1;
      hold(1.8);
    }

    phase("F: autonomous with the trigger held -> expect NOT_TELEOP, robot stops");
    in.autonomous = true;
    hold(3.0);
    in.autonomous = false;

    phase("G: teleop, trigger held, Back = sim pose jump of 0.75 m -> expect POSE_FAULT (latched)");
    hold(1.0);
    tap(BACK);
    hold(3.0);

    phase("H: release trigger, Start = back to the start mark, then hold trigger -> ASSIST");
    in.trigger = 0.0;
    hold(0.5);
    tap(START);
    hold(1.0);
    in.trigger = 1.0;
    hold(8.0);

    phase("I: switch to RED, release, Start, hold -> ASSIST on the red side (mirrored)");
    in.trigger = 0.0;
    in.station = "red1";
    hold(1.0);
    tap(START);
    hold(1.0);
    in.trigger = 1.0;
    hold(15.0);

    phase("J: still red: Shadow (D-pad down) + hold ground (Y), trigger held -> contact, pin timer, rumble, back-off");
    in.pov = 180;
    hold(0.2);
    in.pov = -1;
    tap(Y);
    hold(30.0);

    phase("K: focus cycle with the right bumper -> #0, then back to auto");
    tap(RB);
    hold(3.0);
    tap(RB);
    hold(2.0);

    phase("L: disable -> robot stops");
    in.trigger = 0.0;
    in.enabled = false;
    hold(3.0);
    phase("done");
    ws.sendClose(WebSocket.NORMAL_CLOSURE, "done").join();
  }

  static WebSocket connect() throws InterruptedException {
    HttpClient client = HttpClient.newBuilder().connectTimeout(Duration.ofSeconds(2)).build();
    for (int attempt = 0; attempt < 120; attempt++) {
      try {
        return client
            .newWebSocketBuilder()
            .buildAsync(URI.create("ws://localhost:3300/wpilibws"), new Listener())
            .join();
      } catch (Exception e) {
        Thread.sleep(1000);
      }
    }
    throw new IllegalStateException("could not connect to the robot simulation on port 3300");
  }

  static final class Listener implements WebSocket.Listener {
    private final StringBuilder partial = new StringBuilder();
    private final AtomicReference<String> lastRumble = new AtomicReference<>("");

    @Override
    public CompletionStage<?> onText(WebSocket ws, CharSequence data, boolean last) {
      partial.append(data);
      if (last) {
        String msg = partial.toString();
        partial.setLength(0);
        String r = msg.replaceAll("\\s+", "");
        if (r.contains("\"type\":\"Joystick\"") && r.contains("\"device\":\"0\"")) {
          java.util.regex.Matcher m =
              java.util.regex.Pattern.compile("\"<rumble_left\":(\\d+)").matcher(r);
          if (m.find() && !m.group(1).equals(lastRumble.getAndSet(m.group(1)))) {
            System.out.printf(
                Locale.ROOT, "[script] t=%.1f controller rumble = %s / 65535%n", elapsed(), m.group(1));
          }
        }
      }
      ws.request(1);
      return null;
    }
  }

  static synchronized void send(WebSocket ws) {
    StringBuilder b = new StringBuilder();
    for (int i = 0; i < in.buttons.length; i++) {
      b.append(i == 0 ? "" : ",").append(in.buttons[i]);
    }
    String joystick =
        String.format(
            Locale.ROOT,
            "{\"type\":\"Joystick\",\"device\":\"0\",\"data\":{\">axes\":[%.3f,%.3f,0.0,%.3f,%.3f,0.0],"
                + "\">buttons\":[%s],\">povs\":[%d]}}",
            in.leftX, in.leftY, in.trigger, in.rightX, b, in.pov);
    String ds =
        String.format(
            Locale.ROOT,
            "{\"type\":\"DriverStation\",\"device\":\"\",\"data\":{\">enabled\":%b,\">autonomous\":%b,"
                + "\">test\":false,\">estop\":false,\">fms\":false,\">ds\":true,\">station\":\"%s\","
                + "\">new_data\":true}}",
            in.enabled, in.autonomous, in.station);
    ws.sendText(joystick, true).join();
    ws.sendText(ds, true).join();
  }

  static void tap(int button) throws InterruptedException {
    in.buttons[button] = true;
    hold(0.2);
    in.buttons[button] = false;
    hold(0.2);
  }

  static void hold(double seconds) throws InterruptedException {
    Thread.sleep((long) (seconds * 1000));
  }

  static double elapsed() {
    return (System.nanoTime() - T0) / 1e9;
  }

  static void phase(String text) {
    System.out.printf(Locale.ROOT, "[script] t=%.1f %s%n", elapsed(), text);
  }
}
