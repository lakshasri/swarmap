import React, { useState } from 'react'
import { useRosBridge } from './hooks/useRosBridge'
import ControlPanel from './components/ControlPanel'
import MapCanvas    from './components/MapCanvas'
import StatsPanel   from './components/StatsPanel'
import NetworkGraph from './components/NetworkGraph'
import ReplayPanel  from './components/ReplayPanel'

type ActiveView = 'live' | 'replay'

const styles: Record<string, React.CSSProperties> = {
  root: {
    display: 'flex',
    flexDirection: 'column',
    height: '100vh',
    background: 'var(--bg-primary)',
    color: 'var(--text-primary)',
    overflow: 'hidden',
  },
  header: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: '8px 16px',
    background: 'var(--bg-secondary)',
    borderBottom: '1px solid var(--border)',
    flexShrink: 0,
  },
  title: {
    fontSize: 18,
    fontWeight: 700,
    letterSpacing: 1,
    color: 'var(--accent)',
  },
  connDot: (ok: boolean): React.CSSProperties => ({
    display: 'inline-block',
    width: 10,
    height: 10,
    borderRadius: '50%',
    background: ok ? 'var(--success)' : 'var(--danger)',
    marginRight: 6,
  }),
  connLabel: {
    fontSize: 12,
    color: 'var(--text-muted)',
  },
  tabs: {
    display: 'flex',
    gap: 4,
  },
  tab: (active: boolean): React.CSSProperties => ({
    padding: '4px 14px',
    borderRadius: 4,
    border: '1px solid var(--border)',
    background: active ? 'var(--accent)' : 'transparent',
    color: active ? '#fff' : 'var(--text-muted)',
    cursor: 'pointer',
    fontSize: 12,
  }),
  body: {
    display: 'flex',
    flex: 1,
    overflow: 'hidden',
    minHeight: 0,
  },
  left: {
    width: 240,
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
    width: 320,
    borderLeft: '1px solid var(--border)',
    overflow: 'auto',
    flexShrink: 0,
  },
}

export default function App() {
  const { ros, connected, error } = useRosBridge()
  const [view, setView] = useState<ActiveView>('live')

  return (
    <div style={styles.root}>
      {}
      <header style={styles.header}>
        <span style={styles.title}>SWARMAP</span>
        <div style={styles.tabs}>
          <button style={styles.tab(view === 'live')}   onClick={() => setView('live')}>Live</button>
          <button style={styles.tab(view === 'replay')} onClick={() => setView('replay')}>Replay</button>
        </div>
        <div style={styles.connLabel}>
          <span style={styles.connDot(connected)} />
          {connected ? 'rosbridge connected' : (error ?? 'disconnected')}
        </div>
      </header>

      {}
      <div style={styles.body}>
        {}
        <aside style={styles.left}>
          <ControlPanel ros={ros} />
        </aside>

        {}
        <main style={styles.center}>
          {view === 'live' ? (
            <>
              <MapCanvas ros={ros} style={{ flex: 2 }} />
              <NetworkGraph ros={ros} style={{ flex: 1, borderTop: '1px solid var(--border)' }} />
            </>
          ) : (
            <ReplayPanel />
          )}
        </main>

        {}
        <aside style={styles.right}>
          <StatsPanel ros={ros} />
        </aside>
      </div>
    </div>
  )
}
