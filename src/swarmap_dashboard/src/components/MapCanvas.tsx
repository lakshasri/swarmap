import React, { useRef, useEffect, useCallback, CSSProperties } from 'react'
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
const CELL_FRONTIER = 'rgba(74,158,255,0.7)'

const ROBOT_COLORS = [
  '#4a9eff','#f5a623','#4caf7d','#e05252',
  '#b47aff','#ff6b9d','#00d4aa','#ffdd57',
  '#ff8c42','#6ef4d4',
]

export default function MapCanvas({ ros, style }: Props) {
  const canvasRef  = useRef<HTMLCanvasElement>(null)
  const viewRef    = useRef({ offsetX: 0, offsetY: 0, scale: 4 })
  const dragging   = useRef<{ sx: number; sy: number; ox: number; oy: number } | null>(null)

  const grid  = useRosTopic<OccupancyGrid>(ros, '/swarm/global_map', 'nav_msgs/OccupancyGrid')
  const stats = useRosTopic<DashboardStats>(ros, '/dashboard/stats', 'std_msgs/String')

  
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

    
    let parsedStats: DashboardStats | null = null
    try {
      if (stats) {
        const data = (stats as unknown as { data: string }).data
        parsedStats = JSON.parse(data) as DashboardStats
      }
    } catch {  }

    if (parsedStats && grid) {
      const { width: gw, height: gh, resolution, origin } = grid.info
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
  }, [grid, stats])

  
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
    <canvas
      ref={canvasRef}
      style={{ width: '100%', height: '100%', cursor: 'grab', ...style }}
      onMouseDown={onMouseDown}
      onMouseMove={onMouseMove}
      onMouseUp={onMouseUp}
      onMouseLeave={onMouseUp}
      onWheel={onWheel}
    />
  )
}
