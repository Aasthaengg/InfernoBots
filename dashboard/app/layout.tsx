import './globals.css';
import type { Metadata } from 'next';

export const metadata: Metadata = {
  title: 'Firebot Response Dashboard',
  description: 'Live fire alerts, drone simulation, and data capture'
};

export default function RootLayout({ children }: { children: React.ReactNode }) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  );
}
