'use client';

type RecordRow = {
  id: string;
  type: 'thermal' | 'rgb' | 'telemetry';
  sizeMb: number;
  storedAt: string;
  note?: string;
};

const sample: RecordRow[] = [
  { id: 'frame_18921', type: 'thermal', sizeMb: 3.2, storedAt: '2026-02-15 03:25 UTC', note: 'high heat patch' },
  { id: 'frame_18918', type: 'rgb', sizeMb: 2.8, storedAt: '2026-02-15 03:24 UTC' },
  { id: 'odom_104', type: 'telemetry', sizeMb: 0.4, storedAt: '2026-02-15 03:24 UTC', note: 'waypoint reached' },
  { id: 'frame_18910', type: 'thermal', sizeMb: 3.1, storedAt: '2026-02-15 03:23 UTC' }
];

export default function DataStorePanel({ rows = sample }: { rows?: RecordRow[] }) {
  return (
    <div className="stack">
      <div className="flex" style={{ justifyContent: 'space-between' }}>
        <div className="pill">Data Lake</div>
        <span className="muted">S3-compatible · 30 day retention</span>
      </div>
      <div className="card" style={{ padding: 0 }}>
        <table className="table">
          <thead>
            <tr>
              <th>ID</th>
              <th>Type</th>
              <th>Size</th>
              <th>Stored</th>
              <th>Note</th>
            </tr>
          </thead>
          <tbody>
            {rows.map((r) => (
              <tr key={r.id}>
                <td>{r.id}</td>
                <td>
                  <span className="badge">{r.type}</span>
                </td>
                <td>{r.sizeMb.toFixed(1)} MB</td>
                <td>{r.storedAt}</td>
                <td className="muted">{r.note || '—'}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
}
