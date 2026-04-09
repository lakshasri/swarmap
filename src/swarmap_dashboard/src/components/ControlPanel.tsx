import React, { useState } from 'react'
import ROSLIB from 'roslib'
import { callRosService } from '../hooks/useRosBridge'

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

const SLIDERS: SliderDef[] = [
  { label: 'Swarm size',   param: 'num_robots',    min: 1,   max: 20,  step: 1,    defaultVal: 10,  unit: '' },
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
  btn: (variant: 'primary' | 'danger' | 'default'): React.CSSProperties => ({
    width: '100%',
    padding: '7px 0',
    borderRadius: 6,
    border: 'none',
    cursor: 'pointer',
    fontSize: 12,
    fontWeight: 600,
    background:
      variant === 'primary' ? 'var(--accent)' :
      variant === 'danger'  ? 'var(--danger)' :
                              'var(--bg-secondary)',
    color: '#fff',
  }),
  select: {
    width: '100%',
    padding: '5px 8px',
    background: 'var(--bg-secondary)',
    color: 'var(--text-primary)',
    border: '1px solid var(--border)',
    borderRadius: 6,
    fontSize: 12,
  },
  spinner: { fontSize: 11, color: 'var(--text-muted)', textAlign: 'center' as const },
}

export default function ControlPanel({ ros }: Props) {
  const [values, setValues]         = useState<Record<string, number>>(
    Object.fromEntries(SLIDERS.map(sl => [sl.param, sl.defaultVal]))
  )
  const [failureMode, setFailureMode] = useState('random')
  const [applying, setApplying]       = useState<string | null>(null)

  const sendParam = async (param: string, value: number | string) => {
    if (!ros) return
    setApplying(param)
    try {
      await callRosService(ros, '/swarm/set_param', 'rcl_interfaces/srv/SetParameters', {
        parameters: [{
          name: param,
          value: typeof value === 'number'
            ? { type: 3 , double_value: value }
            : { type: 4 , string_value: value },
        }],
      })
    } catch (e) {
      console.error('set_param failed:', e)
    } finally {
      setApplying(null)
    }
  }

  const sendMissionCmd = async (cmd: string) => {
    if (!ros) return
    
    if (cmd === 'start') {
      await sendParam('exploration_enabled', 1)
    } else if (cmd === 'pause') {
      await sendParam('exploration_enabled', 0)
    } else if (cmd === 'reset') {
      
      console.log('reset requested')
    }
  }

  return (
    <div style={s.root}>
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
          style={s.select}
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
        <button style={s.btn('primary')} onClick={() => sendMissionCmd('start')}>
          Start
        </button>
        <button style={s.btn('default')} onClick={() => sendMissionCmd('pause')}>
          Pause
        </button>
        <button style={s.btn('default')} onClick={() => sendMissionCmd('reset')}>
          Reset
        </button>
        <button style={s.btn('danger')} onClick={() => sendParam('failure_rate', values['failure_rate'])}>
          Inject Failures
        </button>
      </div>
    </div>
  )
}
