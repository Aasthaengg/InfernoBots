'use client';

type DroneStreamProps = {
  streamUrl?: string;
  caption?: string;
};

export default function DroneStream({
  streamUrl = process.env.NEXT_PUBLIC_SIM_STREAM_URL || '',
  caption = 'mavic_2_pro_single_fire.wbt · Webots run'
}: DroneStreamProps) {
  const poster = '/demo/drone-placeholder.svg';
  return (
    <div className="stack">
      <div className="flex" style={{ justifyContent: 'space-between' }}>
        <div className="pill">Drone Simulation</div>
        <span className="muted">{caption}</span>
      </div>
      <div
        style={{
          position: 'relative',
          background: 'linear-gradient(135deg, #1f2937, #0b1220)',
          borderRadius: 14,
          overflow: 'hidden',
          border: '1px solid var(--border)'
        }}
      >
        {streamUrl ? (
          <video
            src={streamUrl}
            poster={poster}
            muted
            autoPlay
            loop
            playsInline
            style={{ display: 'block', width: '100%', height: 360, objectFit: 'cover' }}
          />
        ) : (
          <div
            style={{
              height: 360,
              display: 'grid',
              placeItems: 'center',
              color: 'var(--muted)',
              fontWeight: 600
            }}
          >
            Waiting for Webots stream (set NEXT_PUBLIC_SIM_STREAM_URL)
          </div>
        )}
        <div
          style={{
            position: 'absolute',
            right: 12,
            bottom: 12,
            background: 'rgba(15, 23, 42, 0.72)',
            padding: '6px 10px',
            borderRadius: 10,
            border: '1px solid var(--border)',
            fontSize: 12,
            color: '#e5e7eb',
            display: 'flex',
            alignItems: 'center',
            gap: 8
          }}
        >
          <span className="status-dot" />
          Streaming
        </div>
      </div>
    </div>
  );
}
