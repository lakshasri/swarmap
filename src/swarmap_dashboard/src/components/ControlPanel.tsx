import React, { useState, useMemo } from 'react'
import ROSLIB from 'roslib'
import { useRosTopic } from '../hooks/useRosBridge'
import { robotColor } from '../colors'

interface Props {
  ros: ROSLIB.Ros | null
}

interface RobotRow {
  id: string
  state: string
}

interface Stats {
  robots: RobotRow[]
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
  count: {
    fontSize: 44,
    fontWeight: 200,
    color: 'var(--text)',
    fontFamily: 'var(--font-mono)',
    lineHeight: 1,
  },
  countLabel: {
    fontSize: 10,
    color: 'var(--text-mute)',
    letterSpacing: 1,
    fontFamily: 'var(--font-mono)',
  },
  spawnBtn: {
    width: '100%',
    padding: '10px 12px',
    fontSize: 12,
    letterSpacing: 1.5,
    fontWeight: 600,
    background: 'var(--text)',
    color: 'var(--bg)',
    border: '1px solid var(--text)',
  },
  killBtn: {
    width: '100%',
    padding: '8px 12px',
    fontSize: 11,
    letterSpacing: 1.2,
  },
  select: { width: '100%' },
  sliderRow: { display: 'flex', flexDirection: 'column', gap: 6 },
  sliderLabel: {
    display: 'flex',
    justifyContent: 'space-between',
    fontSize: 11,
    letterSpacing: 1,
    color: 'var(--text-dim)',
    fontFamily: 'var(--font-mono)',
  },
  sliderValue: { color: 'var(--text)', fontWeight: 600 },
  robotList: {
    display: 'flex',
    flexDirection: 'column',
    gap: 2,
    maxHeight: 160,
    overflowY: 'auto',
    border: '1px solid var(--border)',
  },
  robotItem: (selected: boolean): React.CSSProperties => ({
    display: 'flex',
    alignItems: 'center',
    gap: 8,
    padding: '6px 10px',
    fontSize: 11,
    fontFamily: 'var(--font-mono)',
    cursor: 'pointer',
    background: selected ? 'var(--border)' : 'transparent',
    borderLeft: selected ? '2px solid var(--text)' : '2px solid transparent',
  }),
  dot: (col: string): React.CSSProperties => ({
    width: 8,
    height: 8,
    borderRadius: '50%',
    background: col,
    flexShrink: 0,
  }),
  feedback: {
    fontSize: 10,
    color: 'var(--text-mute)',
    letterSpacing: 1,
    fontFamily: 'var(--font-mono)',
    minHeight: 14,
    textAlign: 'center',
    padding: '10px 18px',
  },
}

function publishString(ros: ROSLIB.Ros, topicName: string, data: string) {
  const topic = new ROSLIB.Topic({ ros, name: topicName, messageType: 'std_msgs/String' })
  topic.publish(new ROSLIB.Message({ data }))
}

export default function ControlPanel({ ros }: Props) {
  const [killTarget, setKillTarget]   = useState('')
  const [feedback, setFeedback]       = useState('')
  const [paused, setPaused]           = useState(false)

  const raw = useRosTopic<{ data: string }>(ros, '/dashboard/stats', 'std_msgs/String')
  const robots: RobotRow[] = useMemo(() => {
    if (!raw?.data) return []
    try { return (JSON.parse(raw.data) as Stats).robots ?? [] } catch { return [] }
  }, [raw])
  const activeRobots = robots.filter(r => r.state !== 'FAILED')

  const flash = (msg: string) => {
    setFeedback(msg)
    setTimeout(() => setFeedback(''), 2000)
  }

  const spawnRobot = () => {
    if (!ros) return
    publishString(ros, '/swarm/spawn_robot_request', '')
    flash('SPAWNING…')
  }

  const killRobot = () => {
    if (!ros || !killTarget) return
    publishString(ros, '/swarm/kill_robot_request', killTarget)
    flash(`KILLED ${killTarget.replace('robot_', 'R').toUpperCase()}`)
    setKillTarget('')
  }

  const pauseSim = () => {
    if (!ros) return
    publishString(ros, '/swarm/pause_request', '')
    setPaused(true); flash('PAUSED')
  }
  const resumeSim = () => {
    if (!ros) return
    publishString(ros, '/swarm/resume_request', '')
    setPaused(false); flash('RESUMED')
  }
  const stopSim = () => {
    if (!ros) return
    publishString(ros, '/swarm/stop_request', '')
    setPaused(true); flash('STOPPED')
  }

  const resetMap = () => {
    if (!ros) return
    publishString(ros, '/swarm/reset_map_request', '')
    flash('MAP RESET — rescanning')
  }

  return (
    <div style={s.root}>

      <div style={s.section}>
        <div style={s.heading}>SWARM</div>
        <div style={s.count}>{String(activeRobots.length).padStart(2, '0')}</div>
        <div style={s.countLabel}>ACTIVE ROBOTS</div>
      </div>

      <div style={s.section}>
        <div style={s.heading}>SIMULATION</div>
        <div style={{ display: 'flex', gap: 6 }}>
          {paused ? (
            <button style={{ ...s.spawnBtn, flex: 1, background: '#4ade80', color: '#000', border: '1px solid #4ade80' }} onClick={resumeSim}>RESUME</button>
          ) : (
            <button style={{ ...s.killBtn, flex: 1 }} onClick={pauseSim}>PAUSE</button>
          )}
          <button style={{ ...s.killBtn, flex: 1, color: '#ff5a3c', borderColor: '#ff5a3c' }} onClick={stopSim}>STOP</button>
        </div>
        <button style={{ ...s.killBtn, width: '100%', color: '#ffa500', borderColor: '#ffa500' }} onClick={resetMap}>
          RESET MAP
        </button>
      </div>

      <div style={s.section}>
        <div style={s.heading}>SPAWN / KILL</div>
        <button style={s.spawnBtn} onClick={spawnRobot} disabled={!ros}>
          + SPAWN ROBOT
        </button>

        <div style={{ ...s.heading, marginTop: 4 }}>ACTIVE</div>
        <div style={s.robotList}>
          {activeRobots.length === 0 && (
            <div style={{ padding: '10px', fontSize: 10, color: 'var(--text-mute)', fontFamily: 'var(--font-mono)' }}>
              NO ROBOTS
            </div>
          )}
          {activeRobots.map(r => (
            <div
              key={r.id}
              style={s.robotItem(killTarget === r.id)}
              onClick={() => setKillTarget(killTarget === r.id ? '' : r.id)}
            >
              <span style={s.dot(robotColor(r.id))} />
              <span style={{ color: 'var(--text)' }}>
                {r.id.replace('robot_', 'R').toUpperCase()}
              </span>
              <span style={{ marginLeft: 'auto', color: 'var(--text-mute)', fontSize: 9, letterSpacing: 0.8 }}>
                {r.state}
              </span>
            </div>
          ))}
        </div>

        <button style={s.killBtn} onClick={killRobot} disabled={!killTarget}>
          KILL {killTarget ? killTarget.replace('robot_', 'R').toUpperCase() : 'SELECTED'}
        </button>
      </div>

      <div style={s.feedback}>{feedback || ' '}</div>
    </div>
  )
}
