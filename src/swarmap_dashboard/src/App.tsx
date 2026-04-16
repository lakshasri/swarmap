import React, { useState } from 'react'
import { useRosBridge } from './hooks/useRosBridge'
import ControlPanel from './components/ControlPanel'
import MapCanvas    from './components/MapCanvas'
import StatsPanel   from './components/StatsPanel'
import NetworkGraph from './components/NetworkGraph'

type Tab = 'map' | 'network'

const styles: Record<string, any> = {
  root: {
    display: 'flex',
    flexDirection: 'column',
    height: '100vh',
    background: 'var(--bg)',
    color: 'var(--text)',
    overflow: 'hidden',
  },
  header: {
    display: 'grid',
    gridTemplateColumns: '1fr auto 1fr',
    alignItems: 'center',
    padding: '12px 20px',
    borderBottom: '1px solid var(--border)',
    flexShrink: 0,
  },
  title: {
    fontSize: 14,
    fontWeight: 600,
    letterSpacing: 5,
    color: 'var(--text)',
    fontFamily: 'var(--font-mono)',
    justifySelf: 'start',
  },
  tabs: { display: 'flex', justifySelf: 'center', gap: 0 },
  tab: (active: boolean): React.CSSProperties => ({
    padding: '6px 20px',
    border: '1px solid var(--border-hi)',
    background: active ? 'var(--text)' : 'transparent',
    color: active ? 'var(--bg)' : 'var(--text-dim)',
    fontSize: 11,
    letterSpacing: 1.5,
    cursor: 'pointer',
    fontFamily: 'var(--font-mono)',
  }),
  conn: {
    display: 'flex',
    alignItems: 'center',
    justifySelf: 'end',
    gap: 8,
    fontSize: 11,
    letterSpacing: 1,
    color: 'var(--text-dim)',
    fontFamily: 'var(--font-mono)',
  },
  connDot: (ok: boolean): React.CSSProperties => ({
    width: 8,
    height: 8,
    borderRadius: '50%',
    background: ok ? '#4ade80' : '#3a3a3a',
    border: '1px solid var(--border-hi)',
  }),
  body: {
    display: 'flex',
    flex: 1,
    overflow: 'hidden',
    minHeight: 0,
  },
  left: {
    width: 280,
    borderRight: '1px solid var(--border)',
    overflow: 'auto',
    flexShrink: 0,
  },
  center: {
    flex: 1,
    display: 'flex',
    flexDirection: 'column',
    overflow: 'hidden',
    minWidth: 0,
  },
  right: {
    width: 300,
    borderLeft: '1px solid var(--border)',
    overflow: 'auto',
    flexShrink: 0,
  },
}

export default function App() {
  const { ros, connected, error } = useRosBridge()
  const [tab, setTab] = useState<Tab>('map')

  const tabs: { id: Tab; label: string }[] = [
    { id: 'map',     label: 'MAP' },
    { id: 'network', label: 'NETWORK' },
  ]

  return (
    <div style={styles.root}>
      <header style={styles.header}>
        <span style={styles.title}>SWARMAP</span>
        <div style={styles.tabs}>
          {tabs.map(t => (
            <button
              key={t.id}
              style={styles.tab(tab === t.id)}
              onClick={() => setTab(t.id)}
            >
              {t.label}
            </button>
          ))}
        </div>
        <div style={styles.conn}>
          <span style={styles.connDot(connected)} />
          {connected ? 'CONNECTED' : (error ? 'ERROR' : 'OFFLINE')}
        </div>
      </header>

      <div style={styles.body}>
        <aside style={styles.left}>
          <ControlPanel ros={ros} />
        </aside>

        <main style={styles.center}>
          {tab === 'map'     && <MapCanvas    ros={ros} style={{ flex: 1 }} />}
          {tab === 'network' && <NetworkGraph ros={ros} style={{ flex: 1 }} />}
        </main>

        <aside style={styles.right}>
          <StatsPanel ros={ros} />
        </aside>
      </div>
    </div>
  )
}
