package phantom.core.tracking;

import java.util.List;

/**
 * Everything one camera frame detected. A frame with no detections still matters: it tells the
 * tracker "I looked and saw nothing", which is how missed detections are counted.
 *
 * @param timestampSec when the image was captured (s), which is earlier than when it arrives
 * @param detections anonymous sightings in that image (never null; may be empty)
 */
public record DetectionFrame(double timestampSec, List<Detection> detections) {
  public DetectionFrame {
    detections = List.copyOf(detections);
  }
}
