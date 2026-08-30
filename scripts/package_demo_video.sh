#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: $0 <robot-video.mp4> <presented-video.mp4>" >&2
  exit 2
fi

source_video="$1"
output_video="$2"
font="/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"

if ffmpeg -hide_banner -encoders 2>/dev/null | grep -q h264_nvenc && nvidia-smi >/dev/null 2>&1; then
  encoder=( -c:v h264_nvenc -preset p4 -cq 22 )
else
  encoder=( -c:v libx264 -preset medium -crf 22 )
fi

ffmpeg -y -hide_banner -loglevel warning -i "${source_video}" \
  -f lavfi -t 4 -i "color=c=0x14213d:s=1280x720:r=20" \
  -f lavfi -t 4 -i "color=c=0x14213d:s=1280x720:r=20" \
  -filter_complex \
  "[1:v]drawtext=fontfile=${font}:text=\'Agent request received\':fontcolor=white:fontsize=50:x=(w-text_w)/2:y=250,drawtext=fontfile=${font}:text=\'Query state and process one workpiece through constrained MCP tools\':fontcolor=0x67d5ff:fontsize=27:x=(w-text_w)/2:y=335,format=yuv420p,setpts=PTS-STARTPTS[intro];[0:v]scale=1280:720,fps=20,format=yuv420p,setpts=PTS-STARTPTS[body];[2:v]drawtext=fontfile=${font}:text=\'Physical order completed\':fontcolor=white:fontsize=50:x=(w-text_w)/2:y=250,drawtext=fontfile=${font}:text=\'Inventory and workpiece ownership verified from ROS state\':fontcolor=0x75f0a0:fontsize=30:x=(w-text_w)/2:y=335,format=yuv420p,setpts=PTS-STARTPTS[outro];[intro][body][outro]concat=n=3:v=1:a=0[presented]" \
  -map "[presented]" -an "${encoder[@]}" "${output_video}"

ffprobe -v error -show_entries format=duration,size -of json "${output_video}"
