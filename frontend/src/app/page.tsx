'use client';

import { useEffect, useState } from 'react';

type Status = {
  events: unknown[];
  robots: unknown[];
};

export default function Home() {
  const [status, setStatus] = useState<Status>({ events: [], robots: [] });

  useEffect(() => {
    const refresh = async () => {
      try {
        const res = await fetch('/status', { cache: 'no-store' });
        const data = await res.json();
        setStatus(data);
      } catch (e) {
        // ignore
      }
    };
    refresh();
    const id = setInterval(refresh, 1500);
    return () => clearInterval(id);
  }, []);

  return (
    <main style={{ fontFamily: 'system-ui, sans-serif', margin: '2rem' }}>
      <h1>Laelaps Dashboard (Next.js)</h1>
      <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '1rem' }}>
        <section style={{ border: '1px solid #e5e7eb', borderRadius: 10, padding: '1rem' }}>
          <h2>Events</h2>
          <pre style={{ background: '#f7f7f9', padding: '.75rem', borderRadius: 8, overflow: 'auto' }}>
            {JSON.stringify(status.events, null, 2)}
          </pre>
        </section>
        <section style={{ border: '1px solid #e5e7eb', borderRadius: 10, padding: '1rem' }}>
          <h2>Robots</h2>
          <pre style={{ background: '#f7f7f9', padding: '.75rem', borderRadius: 8, overflow: 'auto' }}>
            {JSON.stringify(status.robots, null, 2)}
          </pre>
        </section>
      </div>
    </main>
  );
}