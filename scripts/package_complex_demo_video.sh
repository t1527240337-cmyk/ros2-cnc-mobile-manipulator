#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 5 ]]; then
  echo "usage: $0 <agent.mp4> <multi-machine.mp4> <recharge.mp4> <reroute.mp4> <output.mp4>" >&2
  exit 2
fi

agent_video="$1"
multi_machine_video="$2"
recharge_video="$3"
reroute_video="$4"
output_video="$5"
font="/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"

for video in "${agent_video}" "${multi_machine_video}" "${recharge_video}" "${reroute_video}"; do
  [[ -s "${video}" ]] || { echo "missing input video: ${video}" >&2; exit 2; }
done

if ffmpeg -hide_banner -encoders 2>/dev/null | grep -q h264_nvenc &&
  nvidia-smi >/dev/null 2>&1; then
  encoder=(-c:v h264_nvenc -preset p4 -cq 22)
else
  encoder=(-c:v libx264 -preset medium -crf 22)
fi

ffmpeg -y -hide_banner -loglevel warning \
  -i "${agent_video}" \
  -i "${multi_machine_video}" \
  -i "${recharge_video}" \
  -i "${reroute_video}" \
  -f lavfi -t 5 -i "color=c=0x14213d:s=1280x720:r=20" \
  -filter_complex \
  "[0:v]scale=1280:720,fps=20,format=yuv420p,setpts=PTS-STARTPTS,drawbox=x=0:y=0:w=iw:h=64:color=0x14213d@0.82:t=fill,drawtext=fontfile=${font}:text='1  Agent -> MCP -> ROS 2 production order':fontcolor=white:fontsize=30:x=28:y=15[a]; \
   [1:v]scale=1280:720,fps=20,format=yuv420p,setpts=PTS-STARTPTS,drawbox=x=0:y=0:w=iw:h=64:color=0x14213d@0.82:t=fill,drawtext=fontfile=${font}:text='2  Multi-CNC scheduling with two concurrent jobs':fontcolor=white:fontsize=30:x=28:y=15[b]; \
   [2:v]scale=1280:720,fps=20,format=yuv420p,setpts=PTS-STARTPTS,drawbox=x=0:y=0:w=iw:h=64:color=0x14213d@0.82:t=fill,drawtext=fontfile=${font}:text='3  Low battery -> dock -> charge -> resume checkpoint':fontcolor=white:fontsize=30:x=28:y=15[c]; \
   [3:v]scale=1280:720,fps=20,format=yuv420p,setpts=PTS-STARTPTS,drawbox=x=0:y=0:w=iw:h=64:color=0x14213d@0.82:t=fill,drawtext=fontfile=${font}:text='4  Preferred CNC fault -> deterministic reassignment':fontcolor=white:fontsize=30:x=28:y=15[d]; \
   [4:v]drawtext=fontfile=${font}:text='Verified physical simulation evidence':fontcolor=white:fontsize=48:x=(w-text_w)/2:y=245,drawtext=fontfile=${font}:expansion=none:text='29/30 fixed seeds | Pick 68/69 | Dock Place Undock 100%':fontcolor=0x75f0a0:fontsize=28:x=(w-text_w)/2:y=330,format=yuv420p,setpts=PTS-STARTPTS[e]; \
   [a][b][c][d][e]concat=n=5:v=1:a=0[out]" \
  -map "[out]" -an "${encoder[@]}" "${output_video}"

ffprobe -v error -show_entries format=duration,size -of json "${output_video}"
