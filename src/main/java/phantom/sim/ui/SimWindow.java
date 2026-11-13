package phantom.sim.ui;

import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.util.HashSet;
import java.util.Set;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.swing.Timer;
import phantom.core.control.ChassisCommand;
import phantom.core.geometry.Vec2;
import phantom.sim.SimSession;

/** Interactive sim window. You drive with the keyboard; hold SPACE to let the assist drive. */
public final class SimWindow extends JPanel {
  private static final double DRIVER_SPEED = 4.5; // m/s at full "stick"
  private static final double DRIVER_TURN = Math.PI; // rad/s

  private final FieldRenderer renderer = new FieldRenderer();
  private final Set<Integer> keysDown = new HashSet<>();
  private SimSession session = SimSession.standard();
  private boolean paused = false;

  public SimWindow() {
    setPreferredSize(new Dimension(1320, 780));
    setFocusable(true);
    addKeyListener(
        new KeyAdapter() {
          @Override
          public void keyPressed(KeyEvent e) {
            keysDown.add(e.getKeyCode());
            switch (e.getKeyCode()) {
              case KeyEvent.VK_P -> paused = !paused;
              case KeyEvent.VK_R -> {
                int mode = session.assist().modeIndex();
                session = SimSession.standard();
                session.assist().setModeIndex(mode);
              }
              case KeyEvent.VK_F -> cycleFocus();
              case KeyEvent.VK_M -> session.assist().nextMode();
              case KeyEvent.VK_1 -> session.assist().setModeIndex(0);
              case KeyEvent.VK_2 -> session.assist().setModeIndex(1);
              case KeyEvent.VK_3 -> session.assist().setModeIndex(2);
              case KeyEvent.VK_4 -> session.assist().setModeIndex(3);
              default -> {}
            }
          }

          @Override
          public void keyReleased(KeyEvent e) {
            keysDown.remove(e.getKeyCode());
          }
        });

    // 20 ms timer = same 50 Hz loop as the robot.
    new Timer(
            (int) Math.round(SimSession.DT * 1000),
            e -> {
              if (!paused) {
                session.step(driverCommand(), keysDown.contains(KeyEvent.VK_SPACE));
              }
              repaint();
            })
        .start();
  }

  /** Cycle focus: auto -> #0 -> #1 -> ... -> auto. */
  private void cycleFocus() {
    var opps = session.world().opponents();
    int current = session.assist().focusOpponentId();
    int idx = -1;
    for (int i = 0; i < opps.size(); i++) {
      if (opps.get(i).body().id() == current) {
        idx = i;
      }
    }
    int next = idx + 1;
    session.assist().setFocusOpponentId(next >= opps.size() ? -1 : opps.get(next).body().id());
  }

  private ChassisCommand driverCommand() {
    double x = (down(KeyEvent.VK_D) ? 1 : 0) - (down(KeyEvent.VK_A) ? 1 : 0);
    double y = (down(KeyEvent.VK_W) ? 1 : 0) - (down(KeyEvent.VK_S) ? 1 : 0);
    double turn = (down(KeyEvent.VK_Q) ? 1 : 0) - (down(KeyEvent.VK_E) ? 1 : 0);
    Vec2 dir = new Vec2(x, y).normalized();
    return new ChassisCommand(dir.times(DRIVER_SPEED), turn * DRIVER_TURN);
  }

  private boolean down(int key) {
    return keysDown.contains(key);
  }

  @Override
  protected void paintComponent(Graphics g) {
    super.paintComponent(g);
    renderer.paint((Graphics2D) g, getWidth(), getHeight(), session, paused);
  }

  public static void open() {
    SwingUtilities.invokeLater(
        () -> {
          JFrame frame = new JFrame("PhantomTrack Sim");
          SimWindow panel = new SimWindow();
          frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
          frame.setContentPane(panel);
          frame.pack();
          frame.setLocationRelativeTo(null);
          frame.setVisible(true);
          panel.requestFocusInWindow();
        });
  }
}
