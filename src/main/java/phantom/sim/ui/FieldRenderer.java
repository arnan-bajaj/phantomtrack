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
import phantom.core.modes.DefenseMode;
import phantom.core.modes.DefenseTarget;
import phantom.core.modes.LineMirrorMode;
import phantom.core.modes.ZoneMode;
import phantom.core.tracking.MultiTargetTracker;
import phantom.core.tracking.TrackedRobot;
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
  private static final Color PARTNER = new Color(0x22a6c9);
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

    // Walls / field elements and the gates between them
    g.setColor(new Color(0x5a, 0x60, 0x6b));
    for (phantom.core.field.Obstacle o : field.obstacles()) {
      Vec2 a = sx(new Vec2(o.min().x(), o.max().y())); // top-left on screen
      g.fill(new Rectangle2D.Double(a.x(), a.y(), (o.max().x() - o.min().x()) * scale,
          (o.max().y() - o.min().y()) * scale));
    }
    g.setColor(new Color(0x8a, 0x91, 0x9c, 120));
    for (Vec2 gate : field.gates()) {
      Vec2 c = sx(gate);
      g.draw(new Ellipse2D.Double(c.x() - 4, c.y() - 4, 8, 8));
    }

    drawModeOverlay(g, assist.activeMode(), field);

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

    // Line from the guarded opponent to the goal (the lane we protect). With anonymous tracking
    // the focus is a track id, so the line starts at that track's estimate.
    boolean anonymous = session.tracking() == SimSession.Tracking.ANONYMOUS;
    if (focusId >= 0 && anonymous) {
      for (TrackedRobot t : assist.lastOpponents()) {
        if (t.id() == focusId) {
          Vec2 a = sx(t.positionAt(world.timeSec()));
          g.setColor(FOCUS);
          Stroke old = g.getStroke();
          g.setStroke(new BasicStroke(1.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10f,
              new float[] {6f, 6f}, 0f));
          g.draw(new Line2D.Double(a.x(), a.y(), goal.x(), goal.y()));
          g.setStroke(old);
        }
      }
    }
    if (focusId >= 0 && !anonymous) {
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

    // Anonymous tracking: tracks that are NOT reported to the defense (tentative, or believed to
    // be a partner) as thin grey outlines.
    MultiTargetTracker mtt = session.multiTracker();
    if (mtt != null) {
      g.setColor(new Color(0x9a, 0xa1, 0xab, 150));
      g.setStroke(new BasicStroke(1f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10f,
          new float[] {2f, 3f}, 0f));
      for (MultiTargetTracker.TrackView t : mtt.tracks()) {
        if (!t.reported()) {
          Vec2 c = sx(t.position().plus(t.velocity().times(world.timeSec() - t.timestampSec())));
          double h = field.robotHalfSize() * scale * 0.8;
          g.draw(new Rectangle2D.Double(c.x() - h, c.y() - h, 2 * h, 2 * h));
        }
      }
    }

    // What the robot believes (only differs from truth with a noisy sensor): hollow outlines
    if (session.sensor() != SimSession.Sensor.PERFECT) {
      g.setColor(new Color(0xff, 0xa0, 0xa0));
      g.setStroke(new BasicStroke(1.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10f, new float[] {4f, 4f}, 0f));
      for (TrackedRobot est : assist.lastOpponents()) {
        Vec2 c = sx(est.positionAt(world.timeSec()));
        double h = field.robotHalfSize() * scale;
        g.draw(new Rectangle2D.Double(c.x() - h, c.y() - h, 2 * h, 2 * h));
        Vec2 tip = sx(est.positionAt(world.timeSec()).plus(est.velocity().times(0.5)));
        g.draw(new Line2D.Double(c.x(), c.y(), tip.x(), tip.y()));
      }
    }

    // Robots
    for (SimWorld.Opponent o : world.opponents()) {
      drawRobot(g, o.body(), field, OPP, !anonymous && o.body().id() == focusId,
          "#" + o.body().id());
    }
    for (SimWorld.Opponent p : world.partners()) {
      drawRobot(g, p.body(), field, PARTNER, false, "ally");
    }

    // Track ids on top of everything (anonymous tracking: ids mean nothing but continuity)
    if (anonymous) {
      g.setColor(new Color(0xff, 0xc0, 0xc0));
      g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 11));
      for (TrackedRobot est : assist.lastOpponents()) {
        Vec2 c = sx(est.positionAt(world.timeSec()));
        double h = field.robotHalfSize() * scale;
        g.drawString("T" + (est.id() - SimSession.FIRST_TRACK_ID), (float) (c.x() + h + 3),
            (float) (c.y() - h + 2));
      }
    }
    drawRobot(g, world.self(), field, SELF, false, "US");

    drawHud(g, width, height, session, focusId, paused);
  }

  /** Shows the geometry the active mode works with: Zone's circle, Line mirror's line. */
  private void drawModeOverlay(Graphics2D g, DefenseMode mode, FieldConfig field) {
    Color overlay = new Color(0x30, 0xc4, 0x6b, 70);
    g.setColor(overlay);
    g.setStroke(new BasicStroke(2f));
    if (mode instanceof ZoneMode zone) {
      Vec2 c = sx(zone.center(field));
      double r = zone.radius() * scale;
      g.fill(new Ellipse2D.Double(c.x() - r, c.y() - r, 2 * r, 2 * r));
      Vec2 a = sx(zone.anchor(field));
      g.setColor(TARGET);
      g.draw(new Rectangle2D.Double(a.x() - 4, a.y() - 4, 8, 8));
    } else if (mode instanceof LineMirrorMode line) {
      Vec2 p0 = line.linePoint(field);
      Vec2 t = line.lineDirection(field);
      double span = Math.hypot(field.length(), field.width());
      Vec2 a = sx(field.clampInside(p0.minus(t.times(span))));
      Vec2 b = sx(field.clampInside(p0.plus(t.times(span))));
      g.setColor(new Color(0x30, 0xc4, 0x6b, 160));
      g.setStroke(new BasicStroke(4f));
      g.draw(new Line2D.Double(a.x(), a.y(), b.x(), b.y()));
    }
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

    boolean anonymous = s.tracking() == SimSession.Tracking.ANONYMOUS;
    String prefix = anonymous ? "T" : "#";
    int offset = anonymous ? SimSession.FIRST_TRACK_ID : 0;
    String focus;
    if (assist.focusOpponentId() == SimSession.NOT_DESIGNATED) {
      focus = "#0, waiting for a track";
    } else if (assist.focusOpponentId() < 0) {
      focus = "auto" + (focusId >= 0 ? " (" + prefix + (focusId - offset) + ")" : "");
    } else {
      focus = prefix + (assist.focusOpponentId() - offset)
          + (s.lockedOpponent() >= 0 ? " (on #" + s.lockedOpponent() + ")" : "");
    }
    String line1 =
        String.format(
            "t = %.1f s   |   Mode: %s   |   Assist: %s   |   Focus: %s   |   Opponents: %s   |   Sensor: %s, %s   |   Partners: %s   |   Field: %s%s",
            world.timeSec(),
            assist.activeMode().name() + (assist.holdEnabled() ? " + hold" : ""),
            assist.lastState(),
            focus,
            world.opponents().isEmpty() ? "-" : world.opponents().get(0).behavior().name(),
            s.sensor().label(),
            s.sensor() == SimSession.Sensor.PERFECT ? "true ids" : s.tracking().label(),
            world.partners().isEmpty() ? "off" : world.partners().size(),
            world.field().name(),
            paused ? "   |   PAUSED" : "");

    StringBuilder cycles = new StringBuilder("Opponent cycles: ");
    for (SimWorld.Opponent o : world.opponents()) {
      cycles.append("#").append(o.body().id()).append(" ").append(o.behavior().cyclesCompleted()).append("   ");
    }
    SimSession.Metrics m = s.metrics();
    String line2 =
        String.format(
            "%s|   Goal-side: %s   |   Tracking error: %s   |   Contact: %.1f s   |   Pins: %d   |   Back-offs: %d",
            cycles,
            Double.isNaN(m.goalSideFraction()) ? "-" : String.format("%.0f%%", 100 * m.goalSideFraction()),
            Double.isNaN(m.meanTrackingError()) ? "-" : String.format("%.2f m", m.meanTrackingError()),
            world.selfContactSeconds(),
            world.pinViolations(),
            assist.backoffsTriggered());
    if (anonymous) {
      line2 += String.format("   |   ID switches: %d   |   Re-points: %d",
          m.idSwitches(), s.redesignations());
    }
    String line3 =
        "WASD drive   J/L turn   SPACE = assist   M / 1-5 = mode   H = hold   F = focus   O = opponents   N = sensor   T = tracking   B = partners   G = field   P = pause   R = reset";

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
