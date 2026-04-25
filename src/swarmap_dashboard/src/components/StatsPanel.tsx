import React, { useMemo, useRef } from 'react'
import ROSLIB from 'roslib'
import { LineChart, Line, XAxis, YAxis, ResponsiveContainer } from 'recharts'
import { useRosTopic } from '../hooks/useRosBridge'
import { robotColor } from '../colors'

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

const s: Record<string, any> = {
  root: { display: 'flex', flexDirection: 'column' },
  section: {
    padding: '16px 18px',
    borderBottom: '1px solid var(--border)',
    display: 'flex',
    flexDirection: 'column',
    gap: 10,
  },
  heading: {
    fontSize: 10,
    fontWeight: 600,
    letterSpacing: 1.8,
    color: 'var(--text-mute)',
    fontFamily: 'var(--font-mono)',
  },
  bigPct: {
    fontSize: 44,
    fontWeight: 200,
    color: 'var(--text)',
    fontFamily: 'var(--font-mono)',
    lineHeight: 1,
  },
  bar: {
    height: 4,
    background: 'var(--border)',
    overflow: 'hidden',
  },
  barFill: (pct: number): React.CSSProperties => ({
    height: '100%',
    background: 'var(--text)',
    width: `${Math.min(100, pct)}%`,
    transition: 'width 0.4s ease',
  }),
  twoCol: { display: 'flex', gap: 32 },
  metric: { display: 'flex', flexDirection: 'column', gap: 4 },
  metricNum: {
    fontSize: 24,
    fontFamily: 'var(--font-mono)',
    color: 'var(--text)',
    fontWeight: 300,
  },
  metricLabel: {
    fontSize: 9,
    color: 'var(--text-mute)',
    letterSpacing: 1.2,
    fontFamily: 'var(--font-mono)',
  },
  robotRow: {
    display: 'grid',
    gridTemplateColumns: '54px 1fr 60px',
    alignItems: 'center',
    gap: 10,
    padding: '8px 0',
    fontSize: 11,
    fontFamily: 'var(--font-mono)',
    borderBottom: '1px solid var(--border)',
  },
  robotIdWrap: { display: 'flex', alignItems: 'center', gap: 6 },
  dot: (col: string): React.CSSProperties => ({
    width: 8,
    height: 8,
    borderRadius: '50%',
    background: col,
    flexShrink: 0,
  }),
  state: { color: 'var(--text-dim)', fontSize: 10, letterSpacing: 0.8 },
  battOuter: {
    height: 6,
    border: '1px solid var(--border-hi)',
    width: 60,
  },
  battInner: (lvl: number): React.CSSProperties => ({
    height: '100%',
    background: lvl > 0.2 ? 'var(--text)' : '#888',
    width: `${Math.max(0, Math.min(1, lvl)) * 100}%`,
  }),
  empty: {
    color: 'var(--text-mute)',
    fontSize: 11,
    fontFamily: 'var(--font-mono)',
    letterSpacing: 0.5,
  },
}

export default function StatsPanel({ ros }: Props) {
  const raw = useRosTopic<{ data: string }>(ros, '/dashboard/stats', 'std_msgs/String')
  const historyRef = useRef<{ t: number; cov: number }[]>([])

  const stats: Stats | null = useMemo(() => {
    if (!raw?.data) return null
    try { return JSON.parse(raw.data) as Stats } catch { return null }
  }, [raw])

  if (stats) {
    historyRef.current.push({ t: Date.now() / 1000, cov: stats.coverage_pct })
    if (historyRef.current.length > 120) historyRef.current.shift()
  }

  const pct    = stats?.coverage_pct  ?? 0
  const active = stats?.active_robots ?? 0
  const failed = stats?.failed_robots ?? 0
  const robots = stats?.robots ?? []

  return (
    <div style={s.root}>

      <div style={s.section}>
        <div style={s.heading}>COVERAGE</div>
        <div style={s.bigPct}>
          {pct.toFixed(1)}
          <span style={{ fontSize: 18, color: 'var(--text-mute)', marginLeft: 2 }}>%</span>
        </div>
        <div style={s.bar}><div style={s.barFill(pct)} /></div>
      </div>

      <div style={s.section}>
        <div style={s.heading}>STATUS</div>
        <div style={s.twoCol}>
          <div style={s.metric}>
            <div style={s.metricNum}>{String(active).padStart(2, '0')}</div>
            <div style={s.metricLabel}>ACTIVE</div>
          </div>
          <div style={s.metric}>
            <div style={s.metricNum}>{String(failed).padStart(2, '0')}</div>
            <div style={s.metricLabel}>FAILED</div>
          </div>
        </div>
      </div>

      <div style={s.section}>
        <div style={s.heading}>COVERAGE OVER TIME</div>
        <div style={{ height: 70 }}>
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={historyRef.current}>
              <XAxis dataKey="t" hide />
              <YAxis domain={[0, 100]} hide />
              <Line
                type="monotone"
                dataKey="cov"
                stroke="#ffffff"
                strokeWidth={1.5}
                dot={false}
                isAnimationActive={false}
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </div>

      <div style={s.section}>
        <div style={s.heading}>ROBOTS</div>
        {robots.length === 0 && <div style={s.empty}>NO DATA</div>}
        {robots.map(r => {
          const failed = r.state === 'FAILED'
          const col = failed ? '#3a3a3a' : robotColor(r.id)
          return (
            <div key={r.id} style={s.robotRow}>
              <span style={s.robotIdWrap}>
                <span style={s.dot(col)} />
                <span style={{ color: 'var(--text)' }}>
                  {r.id.replace('robot_', 'R').toUpperCase()}
                </span>
              </span>
              <span style={s.state}>{r.state}</span>
              <div style={s.battOuter} title={`${(r.battery * 100).toFixed(0)}%`}>
                <div style={s.battInner(r.battery)} />
              </div>
            </div>
          )
        })}
      </div>
    </div>
  )
}
