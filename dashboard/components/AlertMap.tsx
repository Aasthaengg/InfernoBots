'use client';

import dynamic from 'next/dynamic';
import { useEffect, useState } from 'react';
import 'leaflet/dist/leaflet.css';

const LazyMap = dynamic(async () => {
  const mod = await import('react-leaflet');
  return ({ children, ...props }: any) => <mod.MapContainer {...props}>{children}</mod.MapContainer>;
}, { ssr: false });

const LazyTileLayer = dynamic(async () => {
  const mod = await import('react-leaflet');
  return mod.TileLayer;
}, { ssr: false });

const LazyCircleMarker = dynamic(async () => {
  const mod = await import('react-leaflet');
  return mod.CircleMarker;
}, { ssr: false });

const LazyPopup = dynamic(async () => {
  const mod = await import('react-leaflet');
  return mod.Popup;
}, { ssr: false });

type AlertMapProps = {
  center?: [number, number];
  alertRadiusMeters?: number;
  severity?: 'critical' | 'warning' | 'ok';
};

export default function AlertMap({
  center = [37.4275, -122.1697],
  alertRadiusMeters = 120,
  severity = 'critical'
}: AlertMapProps) {
  const [mounted, setMounted] = useState(false);
  useEffect(() => setMounted(true), []);

  if (!mounted) {
    return (
      <div style={{ height: 360, borderRadius: 12, background: 'var(--border)' }} />
    );
  }

  const severityColor = severity === 'critical' ? '#ef4444' : severity === 'warning' ? '#f59e0b' : '#22c55e';

  return (
    <div style={{ position: 'relative' }}>
      <div className="pill" style={{ position: 'absolute', zIndex: 1000, left: 12, top: 12 }}>
        <span className="status-dot" style={{ background: severityColor }} />
        Fire Alert · Live
      </div>
      <LazyMap
        center={center}
        zoom={16}
        scrollWheelZoom={false}
        style={{ height: 360, borderRadius: 12, overflow: 'hidden' }}
      >
        <LazyTileLayer
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
          attribution="© OpenStreetMap"
        />
        <LazyCircleMarker center={center} radius={18} pathOptions={{ color: severityColor, fillColor: severityColor, fillOpacity: 0.25 }}>
          <LazyPopup>
            <b>Fire detected</b>
            <div>Lat {center[0].toFixed(4)}, Lon {center[1].toFixed(4)}</div>
            <div>Evac radius: {alertRadiusMeters} m</div>
          </LazyPopup>
        </LazyCircleMarker>
      </LazyMap>
    </div>
  );
}
