"""
Legacy launcher for the K230 ball tracker.

This keeps the stable camera, tracking, display and RTSP implementation from
K230.py, but restores the UART packet used before frame-age compensation:

    $BALL,SEQ,POS10,PIXEL_X,VALID,QUALITY*CS\r\n

Keep this file in the same directory as K230.py on the CanMV board and run
K230_legacy.py when the old MSPM0 protocol is required.
"""

import K230 as app


def make_legacy_ball_packet(
    sequence,
    position_0p1mm,
    pixel_x,
    valid,
    quality,
    frame_age_ms,
):
    """Build the original six-field BALL packet; frame_age_ms is ignored."""
    payload = "BALL,%04d,%d,%d,%d,%d" % (
        sequence,
        int(position_0p1mm),
        int(pixel_x),
        int(valid),
        int(quality),
    )
    checksum = app.xor_checksum(payload)
    return "$" + payload + "*%02X\r\n" % checksum


# K230.main() resolves this function from its module globals.  Replacing it
# before startup restores the original wire protocol without duplicating the
# large and already-tested vision pipeline.
app.make_ball_packet = make_legacy_ball_packet

app.os.exitpoint(app.os.EXITPOINT_ENABLE)
app.main()
