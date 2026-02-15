import { useEffect, useState } from "react";

const BACKEND = process.env.NEXT_PUBLIC_BACKEND_URL || "http://localhost:8000";
const ALERT_IMAGE =
  "https://cwwp2.dot.ca.gov/data/cache/872_1200x900.jpg"; // replace if needed

export default function Home() {
  const [alerts, setAlerts] = useState([]);
  const [obs, setObs] = useState([]);
  const [loading, setLoading] = useState(false);
  const [note, setNote] = useState("Wildfire detected near Shingletown, CA");

  const fetchData = async () => {
    setLoading(true);
    try {
      const [a, o] = await Promise.all([
        fetch(`${BACKEND}/alerts`).then((r) => r.json()),
        fetch(`${BACKEND}/observations`).then((r) => r.json()),
      ]);
      setAlerts(a);
      setObs(o);
    } catch (e) {
      console.error(e);
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    fetchData();
  }, []);

  const sendAlert = async () => {
    await fetch(`${BACKEND}/alert`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        source: "caltrans_cam",
        location: "Shingletown, CA",
        screenshot_url: ALERT_IMAGE,
        note,
      }),
    });
    fetchData();
  };

  return (
    <main style={{ fontFamily: "Inter, sans-serif", padding: "24px" }}>
      <h1>FireBot — End-to-End Demo</h1>
      <p>
        Alert → Mavic simulation → MindsDB storage. Backend: {BACKEND}
      </p>
      <section style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "20px" }}>
        <div>
          <h3>Incoming Alert</h3>
          <img src={ALERT_IMAGE} alt="alert" style={{ width: "100%", borderRadius: 8 }} />
          <textarea
            value={note}
            onChange={(e) => setNote(e.target.value)}
            style={{ width: "100%", marginTop: 8 }}
          />
          <button onClick={sendAlert} style={{ marginTop: 8 }}>
            Send Alert to Backend
          </button>
        </div>
        <div>
          <h3>How to run simulation</h3>
          <ol>
            <li>Open Webots world: <code>worlds/mavic_2_pro_single_fire.wbt</code></li>
            <li>Controller: <code>mavic_fire_patrol</code></li>
            <li>Run: <code>./run_fire_hover.sh</code></li>
            <li>After hover, POST observation to backend or run your MindsDB pipeline.</li>
          </ol>
          <button onClick={fetchData} disabled={loading}>
            {loading ? "Loading..." : "Refresh data"}
          </button>
          <h4>Alerts</h4>
          <pre style={{ maxHeight: 180, overflow: "auto", background: "#111", color: "#0f0", padding: 8 }}>
            {JSON.stringify(alerts, null, 2)}
          </pre>
          <h4>Observations</h4>
          <pre style={{ maxHeight: 180, overflow: "auto", background: "#111", color: "#0f0", padding: 8 }}>
            {JSON.stringify(obs, null, 2)}
          </pre>
        </div>
      </section>
    </main>
  );
}
