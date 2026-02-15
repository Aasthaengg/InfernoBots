import AlertMap from '../components/AlertMap';
import DroneStream from '../components/DroneStream';
import DataStorePanel from '../components/DataStorePanel';

export default function Home() {
  return (
    <main className="shell stack">
      <header className="stack">
        <div className="pill">Live Incident · #SF-042</div>
        <div className="flex" style={{ justifyContent: 'space-between', alignItems: 'flex-start' }}>
          <div className="stack" style={{ gap: 6 }}>
            <h1>Firebot Response Dashboard</h1>
            <span className="muted">
              Map alert → Webots drone feed from <code>mavic_2_pro_single_fire.wbt</code> → data we store.
            </span>
          </div>
          <button className="btn">Export Report</button>
        </div>
      </header>

      <section className="grid" style={{ gridTemplateColumns: '1.1fr 0.9fr', alignItems: 'start' }}>
        <div className="card">
          <h2 style={{ marginBottom: 12 }}>Fireground Map</h2>
          <AlertMap />
        </div>
        <div className="card">
          <h2 style={{ marginBottom: 12 }}>Drone Live Run</h2>
          <DroneStream />
        </div>
      </section>

      <section className="card">
        <h2 style={{ marginBottom: 12 }}>Captured & Stored</h2>
        <DataStorePanel />
      </section>
    </main>
  );
}
