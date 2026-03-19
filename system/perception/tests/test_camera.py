from src.vision.camera import build_gstreamer_pipeline


def test_build_gstreamer_pipeline_for_udp_uri() -> None:
    pipeline = build_gstreamer_pipeline("udp://0.0.0.0:5000")
    assert "udpsrc address=0.0.0.0 port=5000" in pipeline
    assert "tsparse" in pipeline
    assert "tsdemux" in pipeline
    assert "avdec_h264" in pipeline
    assert "appsink" in pipeline


def test_build_gstreamer_pipeline_for_file() -> None:
    pipeline = build_gstreamer_pipeline("/tmp/test.mp4")
    assert "filesrc location=/tmp/test.mp4" in pipeline
