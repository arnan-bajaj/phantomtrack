package phantom.core.tracking;

import java.util.List;

/**
 * Something that produces anonymous camera frames: a simulated camera now, a real pipeline later.
 * The anonymous counterpart of {@link MeasurementSource}.
 */
public interface DetectionSource {
  /** Frames that arrived since the last call, including empty ones. Never null; may be empty. */
  List<DetectionFrame> poll(double nowSec);
}
