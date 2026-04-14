import React, { useRef, useState, useEffect, useCallback } from 'react'

interface Frame {
  time_s: number
  coverage_pct: number
  active_robots: number
  robots: Array<{ id: string; state: string; battery: number }>
}

interface Mission {
  metadata: { world: string; num_robots: number; recorded_at: string }
  frames: Frame[]
}

const s: Record<string, React.CSSProperties> = {
  root: {
    display: 'flex',
    flexDirection: 'column',
    gap: 16,
    padding: 20,
    height: '100%',
    overflow: 'auto',
    alignItems: 'center',
    justifyContent: 'center',
    color: 'var(--text-primary)',
  },
  dropzone: (active: boolean): React.CSSProperties => ({
    width: '100%',
    maxWidth: 480,
    minHeight: 140,
    border: `2px dashed ${active ? 'var(--accent)' : 'var(--border)'}`,
    borderRadius: 12,
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    justifyContent: 'center',
    gap: 8,
    padding: 24,
    cursor: 'pointer',
    background: active ? 'rgba(74,158,255,0.05)' : 'transparent',
    transition: 'all 0.15s',
  }),
  controls: {
    display: 'flex',
    gap: 10,
    alignItems: 'center',
    flexWrap: 'wrap' as const,
    justifyContent: 'center',
  },
  btn: {
    padding: '6px 14px',
    borderRadius: 6,
    border: '1px solid var(--border)',
    background: 'var(--bg-card)',
    color: 'var(--text-primary)',
    cursor: 'pointer',
    fontSize: 12,
  },
  scrubber: { width: '100%', maxWidth: 480, accentColor: 'var(--accent)' },
  info: {
    fontFamily: 'var(--font-mono)',
    fontSize: 12,
    color: 'var(--text-muted)',
    textAlign: 'center' as const,
  },
}

const SPEEDS = [0.5, 1, 2, 4]

export default function ReplayPanel() {
  const [mission, setMission]       = useState<Mission | null>(null)
  const [frameIdx, setFrameIdx]     = useState(0)
  const [playing, setPlaying]       = useState(false)
  const [speed, setSpeed]           = useState(1)
  const [dragOver, setDragOver]     = useState(false)
  const intervalRef                  = useRef<ReturnType<typeof setInterval> | null>(null)
  const fileInputRef                 = useRef<HTMLInputElement>(null)

  
  useEffect(() => {
    if (intervalRef.current) clearInterval(intervalRef.current)
    if (!playing || !mission) return

    const baseFps = 2  
    const intervalMs = 1000 / (baseFps * speed)
    intervalRef.current = setInterval(() => {
      setFrameIdx(i => {
        if (i >= mission.frames.length - 1) {
          setPlaying(false)
          return i
        }
        return i + 1
      })
    }, intervalMs)

    return () => { if (intervalRef.current) clearInterval(intervalRef.current) }
  }, [playing, speed, mission])

  const loadFile = useCallback((file: File) => {
    const reader = new FileReader()
    reader.onload = (e) => {
      try {
        const data = JSON.parse(e.target?.result as string) as Mission
        setMission(data)
        setFrameIdx(0)
        setPlaying(false)
      } catch {
        alert('Invalid mission JSON file.')
      }
    }
    reader.readAsText(file)
  }, [])

  const onDrop = (e: React.DragEvent) => {
    e.preventDefault()
    setDragOver(false)
    const file = e.dataTransfer.files[0]
    if (file) loadFile(file)
  }

  const frame = mission?.frames[frameIdx]

  return (
    <div style={s.root}>
      {}
      <div
        style={s.dropzone(dragOver)}
        onDragOver={e => { e.preventDefault(); setDragOver(true) }}
        onDragLeave={() => setDragOver(false)}
        onDrop={onDrop}
        onClick={() => fileInputRef.current?.click()}
      >
        <span style={{ fontSize: 32 }}>📂</span>
        <div style={{ fontSize: 14, color: 'var(--text-muted)' }}>
          Drop mission JSON here or click to browse
        </div>
        <input
          ref={fileInputRef}
          type="file"
          accept=".json"
          style={{ display: 'none' }}
          onChange={e => { if (e.target.files?.[0]) loadFile(e.target.files[0]) }}
        />
      </div>

      {mission && (
        <>
          {}
          <div style={s.info}>
            {mission.metadata.world} — {mission.metadata.num_robots} robots
            <br />recorded {mission.metadata.recorded_at}
            <br />{mission.frames.length} frames
          </div>

          {}
          <input
            type="range"
            style={s.scrubber}
            min={0}
            max={mission.frames.length - 1}
            value={frameIdx}
            onChange={e => { setFrameIdx(Number(e.target.value)); setPlaying(false) }}
          />

          {}
          {frame && (
            <div style={s.info}>
              t = {frame.time_s.toFixed(1)} s &nbsp;|&nbsp;
              coverage = {frame.coverage_pct.toFixed(1)}% &nbsp;|&nbsp;
              active = {frame.active_robots}
            </div>
          )}

          {}
          <div style={s.controls}>
            <button style={s.btn} onClick={() => setFrameIdx(0)}>⏮</button>
            <button style={s.btn} onClick={() => setPlaying(p => !p)}>
              {playing ? '⏸' : '▶'}
            </button>
            <button style={s.btn} onClick={() => { setPlaying(false); setFrameIdx(mission.frames.length - 1) }}>⏭</button>
            {SPEEDS.map(sp => (
              <button
                key={sp}
                style={{ ...s.btn, background: speed === sp ? 'var(--accent)' : undefined }}
                onClick={() => setSpeed(sp)}
              >
                {sp}×
              </button>
            ))}
          </div>
        </>
      )}
    </div>
  )
}
