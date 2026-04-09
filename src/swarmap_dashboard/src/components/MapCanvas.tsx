import React, { useRef, useEffect, useCallback, CSSProperties, useState } from 'react'
import ROSLIB from 'roslib'
import { useRosTopic } from '../hooks/useRosBridge'

interface OccupancyGrid {
  info: {
    resolution: number
    width: number
    height: number
    origin: { position: { x: number; y: number } }
  }
  data: number[]
}

interface DashboardStats {
  robots: Array<{ id: string; state: string; battery: number; x: number; y: number }>
}

interface Props {
  ros: ROSLIB.Ros | null
  style?: CSSProperties
}

const CELL_UNKNOWN  = 'rgba(60,62,70,1)'
const CELL_FREE     = 'rgba(220,222,226,1)'
const CELL_OCCUPIED = 'rgba(30,32,40,1)'

const ROBOT_COLORS = [
  '#4a9eff','#f5a623','#4caf7d','#e05252',
  '#b47aff','#ff6b9d','#00d4aa','#ffdd57',
  '#ff8c42','#6ef4d4',
]

function confidenceColor(value: number): string {
  if (value >= 67) return 'rgba(34,197,94,0.55)'   // 3+ visits — green
  if (value >= 34) return 'rgba(6,182,212,0.55)'    // 2 visits  — cyan
  return                   'rgba(59,130,246,0.55)'  // 1 visit   — blue
}

export default function MapCanvas({ ros, style }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const viewRef   = useRef({ offsetX: 0, offsetY: 0, scale: 4 })
  const dragging  = useRef<{ sx: number; sy: number; ox: number; oy: number } | null>(null)

  const [showConf, setShowConf] = useState(false)

  const grid    = useRosTopic<OccupancyGrid>(ros, '/swarm/global_map',          'nav_msgs/OccupancyGrid')
  const confMap = useRosTopic<OccupancyGrid>(ros, '/dashboard/confidence_map',  'nav_msgs/OccupancyGrid')
  const stats   = useRosTopic<DashboardStats>(ros, '/dashboard/stats',          'std_msgs/String')

  const render = useCallback(() => {
    const canvas = canvasRef.current
    if (!canvas || !grid) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return

    const { offsetX, offsetY, scale } = viewRef.current
    const { width, height, resolution } = grid.info

    ctx.clearRect(0, 0, canvas.width, canvas.height)

    for (let gy = 0; gy < height; ++gy) {
      for (let gx = 0; gx < width; ++gx) {
        const val = grid.data[gy * width + gx]
        ctx.fillStyle =
          val === -1  ? CELL_UNKNOWN :
          val === 100 ? CELL_OCCUPIED :
                        CELL_FREE
        ctx.fillRect(
          offsetX + gx * scale,
          offsetY + (height - 1 - gy) * scale,
          scale, scale
        )
      }
    }

    if (showConf && confMap && confMap.data) {
      for (let gy = 0; gy < height; ++gy) {
        for (let gx = 0; gx < width; ++gx) {
          const v = confMap.data[gy * width + gx]
          if (v <= 0) continue
          ctx.fillStyle = confidenceColor(v)
          ctx.fillRect(
            offsetX + gx * scale,
            offsetY + (height - 1 - gy) * scale,
            scale, scale
          )
        }
      }
    }

    let parsedStats: DashboardStats | null = null
    try {
      if (stats) {
        const data = (stats as unknown as { data: string }).data
        parsedStats = JSON.parse(data) as DashboardStats
      }
    } catch { }

    if (parsedStats && grid) {
      const { height: gh, resolution, origin } = grid.info
      const originX = origin.position.x
      const originY = origin.position.y

      const worldToCanvas = (wx: number, wy: number): [number, number] => {
        const gx = (wx - originX) / resolution
        const gy = (wy - originY) / resolution
        return [
          offsetX + gx * scale,
          offsetY + (gh - gy) * scale,
        ]
      }

      parsedStats.robots.forEach((r, i) => {
        const [cx, cy] = worldToCanvas(r.x ?? 0, r.y ?? 0)
        const color = ROBOT_COLORS[i % ROBOT_COLORS.length]

        ctx.beginPath()
        ctx.arc(cx, cy, 5, 0, Math.PI * 2)
        ctx.fillStyle = color
        ctx.fill()
        ctx.strokeStyle = '#fff'
        ctx.lineWidth = 1
        ctx.stroke()

        ctx.fillStyle = '#fff'
        ctx.font = 'bold 9px monospace'
        ctx.textAlign = 'center'
        ctx.fillText(r.id.replace('robot_', 'R'), cx, cy - 8)
      })
    }

    const barCells = Math.round(1.0 / resolution)
    const barPx    = barCells * scale
    ctx.fillStyle = '#fff'
    ctx.fillRect(10, canvas.height - 20, barPx, 3)
    ctx.font = '10px monospace'
    ctx.fillText('1 m', 10 + barPx + 4, canvas.height - 15)

    if (showConf) {
      const items = [
        { color: 'rgba(34,197,94,0.7)',  label: '3+ robots' },
        { color: 'rgba(6,182,212,0.7)',  label: '2 robots'  },
        { color: 'rgba(59,130,246,0.7)', label: '1 robot'   },
      ]
      let lx = 10, ly = 10
      ctx.font = '10px monospace'
      for (const item of items) {
        ctx.fillStyle = item.color
        ctx.fillRect(lx, ly, 10, 10)
        ctx.fillStyle = '#fff'
        ctx.textAlign = 'left'
        ctx.fillText(item.label, lx + 14, ly + 9)
        ly += 16
      }
    }
  }, [grid, confMap, stats, showConf])

  useEffect(() => { render() }, [render])

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const observer = new ResizeObserver(() => {
      canvas.width  = canvas.offsetWidth
      canvas.height = canvas.offsetHeight
      render()
    })
    observer.observe(canvas)
    return () => observer.disconnect()
  }, [render])

  const onMouseDown = (e: React.MouseEvent) => {
    dragging.current = {
      sx: e.clientX, sy: e.clientY,
      ox: viewRef.current.offsetX, oy: viewRef.current.offsetY,
    }
  }
  const onMouseMove = (e: React.MouseEvent) => {
    if (!dragging.current) return
    viewRef.current.offsetX = dragging.current.ox + e.clientX - dragging.current.sx
    viewRef.current.offsetY = dragging.current.oy + e.clientY - dragging.current.sy
    render()
  }
  const onMouseUp = () => { dragging.current = null }

  const onWheel = (e: React.WheelEvent) => {
    e.preventDefault()
    const factor = e.deltaY < 0 ? 1.15 : 0.87
    viewRef.current.scale = Math.max(1, Math.min(32, viewRef.current.scale * factor))
    render()
  }

  return (
    <div style={{ position: 'relative', width: '100%', height: '100%', ...style }}>
      <canvas
        ref={canvasRef}
        style={{ width: '100%', height: '100%', cursor: 'grab', display: 'block' }}
        onMouseDown={onMouseDown}
        onMouseMove={onMouseMove}
        onMouseUp={onMouseUp}
        onMouseLeave={onMouseUp}
        onWheel={onWheel}
      />
      <button
        onClick={() => setShowConf(v => !v)}
        style={{
          position: 'absolute',
          top: 10,
          right: 10,
          padding: '5px 10px',
          borderRadius: 6,
          border: '1px solid var(--border)',
          background: showConf ? 'rgba(59,130,246,0.85)' : 'rgba(30,32,40,0.85)',
          color: '#fff',
          fontSize: 11,
          fontWeight: 700,
          cursor: 'pointer',
          letterSpacing: 0.5,
        }}
      >
        {showConf ? 'CONF ON' : 'CONF OFF'}
      </button>
    </div>
  )
}
