import gi
import numpy as np
import cv2

gi.require_version('Gst', '1.0')
from gi.repository import Gst


def gst_to_opencv(sample):
    """Convert GStreamer sample to OpenCV frame."""
    buf = sample.get_buffer()
    caps = sample.get_caps()

    # Log buffer size and caps
    print(f"Buffer size: {buf.get_size()}")
    print(f"Caps: {caps.to_string()}")

    width = caps.get_structure(0).get_value('width')
    height = caps.get_structure(0).get_value('height')
    expected_size = width * height * 3

    # Check buffer size
    if buf.get_size() != expected_size:
        raise ValueError(f"Buffer size ({buf.get_size()}) does not match expected size ({expected_size})")

    array = np.ndarray(
        (height, width, 3),
        buffer=buf.extract_dup(0, buf.get_size()),
        dtype=np.uint8
    )
    return array


def main():
    Gst.init(None)
    pipeline = Gst.parse_launch(
        'udpsrc port=5600 ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink emit-signals=true sync=false'
    )
    appsink = pipeline.get_by_name("appsink0")
    if not appsink:
        print("Failed to initialize appsink")
        return

    pipeline.set_state(Gst.State.PLAYING)

    try:
        while True:
            sample = appsink.emit("pull-sample")
            if sample:
                frame = gst_to_opencv(sample)
                cv2.imshow("GStreamer Frame", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print("No sample received.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        pipeline.set_state(Gst.State.NULL)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
