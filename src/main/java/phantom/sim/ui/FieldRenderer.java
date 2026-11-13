package phantom.sim.ui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.Stroke;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.Optional;
import phantom.core.field.FieldConfig;
import phantom.core.geometry.Vec2;
import phantom.core.manager.DefenseAssist;
import phantom.core.modes.DefenseTarget;
import phantom.sim.RobotBody;
import phantom.sim.SimSession;
import phantom.sim.SimWorld;

/** Draws the field, robots, defense target and HUD. Shared by the window and PNG snapshots. */
public final class FieldRenderer {
  private static final Color BG = new Color(0x1b1d21);
  private static final Color CARPET = new Color(0x2a2f36);
  private static final Color LINES = new Color(0x8a919c);
  private static final Color SELF = new Color(0x3b82f6);
  private static final Color OPP = new Color(0xe5484d);
  private static final Color FOCUS = new Color(0xf5c518);
  private static final Color TARGET = new Color(0x30c46b);
  private static final Color GOAL = new Color(0xf08c2e);
  private static final Color TEXT = new Color(0xe6e8eb);
  private static final Color MUTED = new Color(0x9aa1ab);
  private static final int MARGIN = 24;
  private static final int HUD_HEIGHT = 78;

  private double scale;
  private double originX;
  private double originY;
  private double fieldWidth;

  public void paint(Graphics2D g, int width, int height, SimSession session, boolean paused) {
    g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
    g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
    SimWorld world = session.world();
    FieldConfig field = world.field();
    DefenseAssist assist = session.assist();
    Optional<DefenseTarget> target = assist.lastTarget();
    int focusId = target.map(DefenseTarget::opponentId).orElse(-1);

    g.setColor(BG);
    g.fillRect(0, 0, width, height);

    double availW = width - 2.0 * MARGIN;
    double availH = height - 2.0 * MARGIN - HUD_HEIGHT;
    scale = Math.min(availW / field.length(), availH / field.width());
    fieldWidth = field.width();
    originX = (width - field.length() * scale) / 2.0;
    originY = MARGIN;

    // Field
    g.setColor(CARPET);
    g.fill(new Rectangle2D.Double(originX, originY, field.length() * scale, field.width() * scale));
    g.setColor(LINES);
    g.setStroke(new BasicStroke(2f));
    g.draw(new Rectangle2D.Double(originX, originY, field.length() * scale, field.width() * scale));
    Vec2 mid1 = sx(new Vec2(field.length() / 2, 0));
    Vec2 mid2 = sx(new Vec2(field.length() / 2, field.width()));
    g.setStroke(new BasicStroke(1f));
    g.draw(new Line2D.Double(mid1.x(), mid1.y(), mid2.x(), mid2.y()));

    // Pickups and goal
    g.setColor(MUTED);
    for (Vec2 p : field.opponentPickups()) {
      Vec2 s = sx(p);
      double r = 0.3 * scale;
      g.draw(new Rectangle2D.Double(s.x() - r, s.y() - r, 2 * r, 2 * r));
    }
    Vec2 goal = sx(field.defendedGoal());
    g.setColor(GOAL);
    double gr = 0.35 * scale;
    g.fill(new Ellipse2D.Double(goal.x() - gr, goal.y() - gr, 2 * gr, 2 * gr));
    g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 12));
    g.drawString("GOAL", (float) (goal.x() - 16), (float) (goal.y() - gr - 6));

    // Line from the guarded opponent to the goal (the lane we protect)
    if (focusId >= 0) {
      for (SimWorld.Opponent o : world.opponents()) {
        if (o.body().id() == focusId) {
          Vec2 a = sx(o.body().position());
          g.setColor(FOCUS);
          Stroke old = g.getStroke();
          g.setStroke(
              new BasicStroke(1.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10f, new float[] {6f, 6f}, 0f));
          g.draw(new Line2D.Double(a.x(), a.y(), goal.x(), goal.y()));
          g.setStroke(old);
        }
      }
    }

    // Defense target
    if (target.isPresent()) {
      Vec2 t = sx(target.get().position());
      Vec2 me = sx(world.self().position());
      g.setColor(TARGET);
      g.setStroke(new BasicStroke(2f));
      double tr = 0.22 * scale;
      g.draw(new Ellipse2D.Double(t.x() - tr, t.y() - tr, 2 * tr, 2 * tr));
      g.draw(new Line2D.Double(t.x() - tr * 0.6, t.y(), t.x() + tr * 0.6, t.y()));
      g.draw(new Line2D.Double(t.x(), t.y() - tr * 0.6, t.x(), t.y() + tr * 0.6));
      g.setStroke(new BasicStroke(1f));
      g.draw(new Line2D.Double(me.x(), me.y(), t.x(), t.y()));
    }

    // Robots
    for (SimWorld.Opponent o : world.opponents()) {
      drawRobot(g, o.body(), field, OPP, o.body().id() == focusId, "#" + o.body().id());
    }
    drawRobot(g, world.self(), field, SELF, false, "US");

    drawHud(g, width, height, session, focusId, paused);
  }

  private void drawRobot(
      Graphics2D g, RobotBody body, FieldConfig field, Color color, boolean focus, String label) {
    Vec2 c = sx(body.position());
    double half = field.robotHalfSize() * scale;
    AffineTransform saved = g.getTransform();
    g.translate(c.x(), c.y());
    g.rotate(-body.heading()); // screen y is flipped, so rotate the other way
    g.setColor(color);
    g.fill(new Rectangle2D.Double(-half, -half, 2 * half, 2 * half));
    g.setColor(focus ? FOCUS : color.darker());
    g.setStroke(new BasicStroke(focus ? 3f : 2f));
    g.draw(new Rectangle2D.Double(-half, -half, 2 * half, 2 * half));
    // Front marker
    g.setColor(Color.WHITE);
    g.setStroke(new BasicStroke(3f));
    g.draw(new Line2D.Double(0, 0, half * 0.9, 0));
    g.setTransform(saved);

    g.setColor(Color.WHITE);
    g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 12));
    int tw = g.getFontMetrics().stringWidth(label);
    g.drawString(label, (float) (c.x() - tw / 2.0), (float) (c.y() + half + 14));
  }

  private void drawHud(
      Graphics2D g, int width, int height, SimSession s, int focusId, boolean paused) {
    SimWorld world = s.world();
    DefenseAssist assist = s.assist();
    int y0 = height - HUD_HEIGHT + 4;

    String focus =
        assist.focusOpponentId() < 0
            ? "auto" + (focusId >= 0 ? " (#" + focusId + ")" : "")
            : "#" + assist.focusOpponentId();
    String line1 =
        String.format(
            "t = %.1f s   |   Mode: %s   |   Assist: %s   |   Focus: %s%s",
            world.timeSec(),
            assist.activeMode().name(),
            assist.lastState(),
            focus,
            paused ? "   |   PAUSED" : "");

    StringBuilder cycles = new StringBuilder("Opponent cycles: ");
    for (SimWorld.Opponent o : world.opponents()) {
      cycles.append("#").append(o.body().id()).append(" ").append(o.behavior().cyclesCompleted()).append("   ");
    }
    SimSession.Metrics m = s.metrics();
    String line2 =
        String.format(
            "%s|   Goal-side: %s   |   Tracking error: %s   |   Contact: %.1f s",
            cycles,
            Double.isNaN(m.goalSideFraction()) ? "-" : String.format("%.0f%%", 100 * m.goalSideFraction()),
            Double.isNaN(m.meanTrackingError()) ? "-" : String.format("%.2f m", m.meanTrackingError()),
            world.selfContactSeconds());
    String line3 =
        "WASD drive   Q/E rotate   hold SPACE = assist   F = cycle focus   P = pause   R = reset";

    g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 14));
    g.setColor(TEXT);
    g.drawString(line1, MARGIN, y0 + 16);
    g.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 13));
    g.drawString(line2, MARGIN, y0 + 38);
    g.setColor(MUTED);
    g.drawString(line3, MARGIN, y0 + 60);
  }

  /** Field meters -> screen pixels (y flipped so +y is up). */
  private Vec2 sx(Vec2 p) {
    return new Vec2(originX + p.x() * scale, originY + (fieldWidth - p.y()) * scale);
  }
}
