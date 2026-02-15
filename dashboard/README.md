# Firebot Dashboard (Vercel-ready)

Single-page Next.js app that shows:
- Map with a live fire alert footprint.
- Drone simulation pane sourced from `mavic_2_pro_single_fire.wbt` (Webots stream URL).
- Table of captured/stored data.

## Quick start
```bash
cd dashboard
npm install          # or pnpm/yarn
npm run dev          # http://localhost:3000
```

## Configure the live drone feed
Set an MJPEG/MP4/HLS endpoint exposed by your Webots run (e.g. rosbridge + web_video_server or ffmpeg) in an env var:
```
NEXT_PUBLIC_SIM_STREAM_URL=http://localhost:8080/stream.mjpeg
```
On Vercel, add this in Project Settings → Environment Variables.

## Deploy on Vercel
- Push `dashboard/` to your repo, create a Vercel project pointing to that subdirectory.
- Framework: Next.js. Build command: `npm run build`. Output dir: `.next`.

## Notes
- Map uses OpenStreetMap tiles (no token needed).
- If you change the incident location, edit `center` in `app/page.tsx` or pass it via props/state once you have a live backend.
