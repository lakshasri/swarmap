import React, { useMemo } from 'react'
import ROSLIB from 'roslib'
import { LineChart, Line, XAxis, YAxis, Tooltip, ResponsiveContainer } from 'recharts'
import { useRosTopic } from '../hooks/useRosBridge'

interface RobotRow {
  id: string
  state: string
  battery: number
  cells_mapped: number
}

interface Stats {
  coverage_pct: number
  active_robots: number
  failed_robots: number
  robots: RobotRow[]
}

interface Props {
  ros: ROSLIB.Ros | null
}

const s: Record<string, React.CSSProperties> = {
  root: {
    padding: 12,
    display: 'flex',
    flexDirection: 'column',
    gap: 16,
    height: '100%',
    overflow: 'auto',
  },
  section: {
    background: 'var(--bg-card)',
    borderRadius: 8,
    padding: 12,
    border: '1px solid var(--border)',
  },
  label: { fontSize: 11, color: 'var(--text-muted)', marginBottom: 4 },
  bigNum: { fontSize: 32, fontWeight: 700, color: 'var(--accent)' },
  ring: (pct: number): React.CSSProperties => ({
    width: 80,
    height: 80,
    borderRadius: '50%',
    background: `conic-gradient(var(--accent) ${pct * 3.6}deg, var(--bg-secondary) 0deg)`,
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    fontSize: 14,
    fontWeight: 700,
    color: 'var(--text-primary)',
  }),
  row: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 4,
  },
  battery: (lvl: number): React.CSSProperties => ({
    width: 50,
    height: 8,
    borderRadius: 4,
    background: `linear-gradient(to right,
      ${lvl > 0.3 ? 'var(--success)' : 'var(--danger)'} ${lvl * 100}%,
      var(--bg-secondary) 0%)`,
    border: '1px solid var(--border)',
  }),
  stateChip: (state: string): React.CSSProperties => ({
    fontSize: 10,
    padding: '2px 6px',
    borderRadius: 10,
    background:
      state === 'EXPLORING'  ? 'var(--accent)' :
      state === 'NAVIGATING' ? '#6a5acd' :
      state === 'RETURNING'  ? 'var(--warning)' :
      state === 'FAILED'     ? 'var(--danger)' :
                               'var(--bg-secondary)',
    color: '#fff',
  }),
}

const history: { t: number; cov: number }[] = []

export default function StatsPanel({ ros }: Props) {
  const raw = useRosTopic<{ data: string }>(ros, '/dashboard/stats', 'std_msgs/String')

  const stats: Stats | null = useMemo(() => {
    if (!raw?.data) return null
    try { return JSON.parse(raw.data) as Stats } catch { return null }
  }, [raw])

  
  if (stats) {
    history.push({ t: Date.now() / 1000, cov: stats.coverage_pct })
    if (history.length > 120) history.shift()
  }

  const pct   = stats?.coverage_pct   ?? 0
  const active= stats?.active_robots  ?? 0
  const failed= stats?.failed_robots  ?? 0
  const robots= stats?.robots         ?? []

  return (
    <div style={s.root}>

      {}
      <div style={s.section}>
        <div style={s.label}>COVERAGE</div>
        <div style={{ display: 'flex', alignItems: 'center', gap: 16 }}>
          <div style={s.ring(pct)}>{pct.toFixed(1)}%</div>
          <div>
            <div style={{ color: 'var(--success)', fontSize: 13 }}>
              {active} active
            </div>
            <div style={{ color: 'var(--danger)', fontSize: 13 }}>
              {failed} failed
            </div>
          </div>
        </div>
      </div>

      {}
      <div style={{ ...s.section, height: 120 }}>
        <div style={s.label}>COVERAGE OVER TIME</div>
        <ResponsiveContainer width="100%" height={88}>
          <LineChart data={history}>
            <XAxis dataKey="t" hide />
            <YAxis domain={[0, 100]} hide />
            <Tooltip formatter={(v: number) => `${v.toFixed(1)}%`} />
            <Line type="monotone" dataKey="cov" stroke="var(--accent)" dot={false} />
          </LineChart>
        </ResponsiveContainer>
      </div>

      {}
      <div style={s.section}>
        <div style={s.label}>ROBOTS</div>
        {robots.map((r) => (
          <div key={r.id} style={s.row}>
            <span style={{ fontSize: 12, fontFamily: 'var(--font-mono)' }}>
              {r.id}
            </span>
            <span style={s.stateChip(r.state)}>{r.state}</span>
            <div style={s.battery(r.battery)} title={`${(r.battery * 100).toFixed(0)}%`} />
          </div>
        ))}
        {robots.length === 0 && (
          <div style={{ color: 'var(--text-muted)', fontSize: 12 }}>
            Waiting for robots…
          </div>
        )}
      </div>

    </div>
  )
}
