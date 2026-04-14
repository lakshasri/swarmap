import React, { useState, useMemo } from 'react'
import ROSLIB from 'roslib'
import { callRosService, useRosTopic } from '../hooks/useRosBridge'

interface Props {
  ros: ROSLIB.Ros | null
}

interface SliderDef {
  label: string
  param: string
  min: number
  max: number
  step: number
  defaultVal: number
  unit: string
}

interface RobotRow {
  id: string
  state: string
}

interface Stats {
  robots: RobotRow[]
}

const SLIDERS: SliderDef[] = [
  { label: 'Sensor range', param: 'sensor_range',  min: 2,   max: 15,  step: 0.5,  defaultVal: 5,   unit: 'm' },
  { label: 'Noise level',  param: 'noise_level',   min: 0,   max: 1,   step: 0.05, defaultVal: 0,   unit: '' },
  { label: 'Failure rate', param: 'failure_rate',  min: 0,   max: 0.45,step: 0.01, defaultVal: 0,   unit: '/min' },
  { label: 'Comm radius',  param: 'comm_radius',   min: 2,   max: 20,  step: 0.5,  defaultVal: 8,   unit: 'm' },
]

const FAILURE_MODES = ['random', 'progressive', 'cascade']

const s: Record<string, React.CSSProperties> = {
  root: { padding: 12, display: 'flex', flexDirection: 'column', gap: 12 },
  heading: { fontSize: 11, fontWeight: 700, color: 'var(--text-muted)', letterSpacing: 1 },
  section: {
    background: 'var(--bg-card)',
    border: '1px solid var(--border)',
    borderRadius: 8,
    padding: 10,
    display: 'flex',
    flexDirection: 'column',
    gap: 10,
  },
  sliderRow: { display: 'flex', flexDirection: 'column', gap: 3 },
  sliderLabel: { display: 'flex', justifyContent: 'space-between', fontSize: 12 },
  slider: { width: '100%', accentColor: 'var(--accent)' },
  btn: (variant: 'primary' | 'danger' | 'success' | 'default'): React.CSSProperties => ({
    flex: 1,
    padding: '8px 0',
    borderRadius: 6,
    border: 'none',
    cursor: 'pointer',
    fontSize: 13,
    fontWeight: 700,
    background:
      variant === 'primary' ? 'var(--accent)' :
      variant === 'danger'  ? 'var(--danger)' :
      variant === 'success' ? 'var(--success)' :
                              'var(--bg-secondary)',
    color: '#fff',
  }),
  select: {
    flex: 1,
    padding: '7px 8px',
    background: 'var(--bg-secondary)',
    color: 'var(--text-primary)',
    border: '1px solid var(--border)',
    borderRadius: 6,
    fontSize: 12,
  },
  row: { display: 'flex', gap: 8, alignItems: 'center' },
  spinner: { fontSize: 11, color: 'var(--text-muted)', textAlign: 'center' as const },
  count: {
    fontSize: 28,
    fontWeight: 700,
    color: 'var(--accent)',
    textAlign: 'center' as const,
    fontFamily: 'var(--font-mono)',
  },
  countLabel: { fontSize: 11, color: 'var(--text-muted)', textAlign: 'center' as const },
  feedback: (ok: boolean): React.CSSProperties => ({
    fontSize: 11,
    color: ok ? 'var(--success)' : 'var(--danger)',
    textAlign: 'center' as const,
  }),
}

function publishString(ros: ROSLIB.Ros, topicName: string, data: string) {
  const topic = new ROSLIB.Topic({ ros, name: topicName, messageType: 'std_msgs/String' })
  topic.publish(new ROSLIB.Message({ data }))
}

export default function ControlPanel({ ros }: Props) {
  const [values, setValues] = useState<Record<string, number>>(
    Object.fromEntries(SLIDERS.map(sl => [sl.param, sl.defaultVal]))
  )
  const [failureMode, setFailureMode] = useState('random')
  const [applying, setApplying]       = useState<string | null>(null)
  const [killTarget, setKillTarget]   = useState('')
  const [feedback, setFeedback]       = useState<{ msg: string; ok: boolean } | null>(null)

  const raw = useRosTopic<{ data: string }>(ros, '/dashboard/stats', 'std_msgs/String')
  const robots: RobotRow[] = useMemo(() => {
    if (!raw?.data) return []
    try { return (JSON.parse(raw.data) as Stats).robots ?? [] } catch { return [] }
  }, [raw])

  const flash = (msg: string, ok = true) => {
    setFeedback({ msg, ok })
    setTimeout(() => setFeedback(null), 2500)
  }

  const sendParam = async (param: string, value: number | string) => {
    if (!ros) return
    setApplying(param)
    try {
      await callRosService(ros, '/swarm/set_param', 'rcl_interfaces/srv/SetParameters', {
        parameters: [{
          name: param,
          value: typeof value === 'number'
            ? { type: 3, double_value: value }
            : { type: 4, string_value: value },
        }],
      })
    } catch (e) {
      console.error('set_param failed:', e)
    } finally {
      setApplying(null)
    }
  }

  const spawnRobot = () => {
    if (!ros) return
    publishString(ros, '/swarm/spawn_robot', '{}')
    flash('Spawning robot…')
  }

  const killRobot = () => {
    if (!ros || !killTarget) return
    publishString(ros, '/swarm/kill_robot', killTarget)
    flash(`Killed ${killTarget}`, true)
    setKillTarget('')
  }

  const sendMissionCmd = async (cmd: string) => {
    if (!ros) return
    if (cmd === 'start') await sendParam('exploration_enabled', 1)
    else if (cmd === 'pause') await sendParam('exploration_enabled', 0)
  }

  return (
    <div style={s.root}>

      <div style={s.heading}>ROBOT MANAGEMENT</div>
      <div style={s.section}>
        <div style={s.count}>{robots.length}</div>
        <div style={s.countLabel}>robots active</div>

        <button style={{ ...s.btn('success'), flex: 'unset', width: '100%', fontSize: 14 }} onClick={spawnRobot}>
          + Spawn Robot
        </button>

        <div style={s.row}>
          <select
            style={s.select}
            value={killTarget}
            onChange={e => setKillTarget(e.target.value)}
          >
            <option value=''>Select robot to kill…</option>
            {robots.map(r => (
              <option key={r.id} value={r.id}>
                {r.id}  [{r.state}]
              </option>
            ))}
          </select>
          <button
            style={{ ...s.btn('danger'), flex: 'unset', padding: '8px 14px' }}
            onClick={killRobot}
            disabled={!killTarget}
          >
            Kill
          </button>
        </div>

        {feedback && <div style={s.feedback(feedback.ok)}>{feedback.msg}</div>}
      </div>

      <div style={s.heading}>PARAMETERS</div>
      <div style={s.section}>
        {SLIDERS.map(sl => (
          <div key={sl.param} style={s.sliderRow}>
            <div style={s.sliderLabel}>
              <span>{sl.label}</span>
              <span style={{ color: 'var(--accent)', fontFamily: 'var(--font-mono)' }}>
                {values[sl.param]}{sl.unit}
              </span>
            </div>
            <input
              type="range"
              style={s.slider}
              min={sl.min}
              max={sl.max}
              step={sl.step}
              value={values[sl.param]}
              onChange={e => setValues(v => ({ ...v, [sl.param]: parseFloat(e.target.value) }))}
              onMouseUp={() => sendParam(sl.param, values[sl.param])}
              onTouchEnd={() => sendParam(sl.param, values[sl.param])}
            />
            {applying === sl.param && <div style={s.spinner}>Applying…</div>}
          </div>
        ))}
      </div>

      <div style={s.heading}>FAILURE MODE</div>
      <div style={s.section}>
        <select
          style={{ ...s.select, flex: 'unset' }}
          value={failureMode}
          onChange={e => {
            setFailureMode(e.target.value)
            sendParam('failure_mode', e.target.value)
          }}
        >
          {FAILURE_MODES.map(m => (
            <option key={m} value={m}>{m}</option>
          ))}
        </select>
      </div>

      <div style={s.heading}>MISSION CONTROL</div>
      <div style={s.section}>
        <div style={s.row}>
          <button style={s.btn('primary')} onClick={() => sendMissionCmd('start')}>Start</button>
          <button style={s.btn('default')} onClick={() => sendMissionCmd('pause')}>Pause</button>
        </div>
        <button style={{ ...s.btn('danger'), flex: 'unset', width: '100%' }}
          onClick={() => sendParam('failure_rate', values['failure_rate'])}>
          Inject Failures
        </button>
      </div>

    </div>
  )
}
