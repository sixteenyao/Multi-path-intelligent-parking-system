#!/bin/bash
set -e
BASE="/home/sixteen/VIRAT_Datasets"
TOOLS="$(cd "$(dirname "$0")/.." && pwd)/tools"

echo "=== 启动 MediamTX ==="
pkill mediamtx 2>/dev/null || true
pkill ffmpeg 2>/dev/null || true
sleep 1

cat > /tmp/mediamtx.yml <<'EOF'
rtmp: no
hls: no
webrtc: no
paths:
  cam1:
  cam2:
  cam3:
  cam4:
EOF

"$TOOLS/mediamtx" /tmp/mediamtx.yml &
sleep 3

sleep 1
rtsp_port=$(ss -tlnp 2>/dev/null | grep 8554 || netstat -tlnp 2>/dev/null | grep 8554)
[ -n "$rtsp_port" ] && echo "✅ MediamTX started on :8554" || { echo "❌ MediamTX failed"; exit 1; }

echo ""
echo "=== 启动 4 路推流 ==="

for cam in 1 2 3 4; do
    FOLDER_NUM=$(printf "%02d" $((cam-1)))
    FOLDER="$BASE/videos-$FOLDER_NUM/videos_original"
    PLAYLIST="/tmp/cam${cam}.txt"

    ls "$FOLDER"/*.mp4 | sort | sed "s/.*/file '&'/" > "$PLAYLIST"
    count=$(wc -l < "$PLAYLIST")

    echo -n "cam$cam: $count videos → "

    ffmpeg -hide_banner -loglevel error \
        -re -f concat -safe 0 -stream_loop -1 \
        -i "$PLAYLIST" \
        -c copy -f rtsp "rtsp://127.0.0.1:8554/cam$cam" &

    sleep 1
    echo "started"
done

sleep 2
echo ""
echo "=== 4 路 RTSP 就绪 ==="
echo "  rtsp://127.0.0.1:8554/cam1 (A区)"
echo "  rtsp://127.0.0.1:8554/cam2 (B区)"
echo "  rtsp://127.0.0.1:8554/cam3 (C区)"
echo "  rtsp://127.0.0.1:8554/cam4 (D区)"
echo ""
echo "验证: ffplay rtsp://127.0.0.1:8554/cam1"
