import { useRef, useEffect, useCallback, CSSProperties } from 'react'
import ROSLIB from 'roslib'
import { useRosTopic } from '../hooks/useRosBridge'
import { robotColor } from '../colors'

interface GraphNode {
  id: string
  x: number
  y: number
  failed: boolean
}
interface GraphEdge {
  source: string
  target: string
}
interface Topology {
  nodes: GraphNode[]
  edges: GraphEdge[]
}

interface OccupancyInfo {
  info: {
    resolution: number
    width: number
    height: number
    origin: { position: { x: number; y: number } }
  }
}

interface Props {
  ros: ROSLIB.Ros | null
  style?: CSSProperties
}

const WORLD_W_M = 50
const WORLD_H_M = 50

export default function NetworkGraph({ ros, style }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const raw = useRosTopic<{ data: string }>(ros, '/dashboard/network_topology', 'std_msgs/String')
  const grid = useRosTopic<OccupancyInfo>(ros, '/swarm/global_map', 'nav_msgs/OccupancyGrid')

  const render = useCallback(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return

    ctx.fillStyle = '#0a0b0d'
    ctx.fillRect(0, 0, canvas.width, canvas.height)

    let topo: Topology | null = null
    try {
      if (raw?.data) topo = JSON.parse(raw.data) as Topology
    } catch { return }

    if (!topo || topo.nodes.length === 0) {
      ctx.fillStyle = '#5a5a5a'
      ctx.font = '11px ui-monospace, monospace'
      ctx.textAlign = 'center'
      ctx.fillText('WAITING FOR TOPOLOGY', canvas.width / 2, canvas.height / 2)
      return
    }

    const worldW = grid?.info
      ? grid.info.width * grid.info.resolution
      : WORLD_W_M
    const worldH = grid?.info
      ? grid.info.height * grid.info.resolution
      : WORLD_H_M
    const originX = grid?.info?.origin.position.x ?? 0
    const originY = grid?.info?.origin.position.y ?? 0

    const pad = 40
    const availW = canvas.width  - 2 * pad
    const availH = canvas.height - 2 * pad
    const scale = Math.min(availW / worldW, availH / worldH)
    const offsetX = (canvas.width  - worldW * scale) / 2
    const offsetY = (canvas.height - worldH * scale) / 2

    const toCanvas = (wx: number, wy: number): [number, number] => [
      offsetX + (wx - originX) * scale,
      offsetY + (worldH - (wy - originY)) * scale,
    ]

    ctx.strokeStyle = 'rgba(255,255,255,0.08)'
    ctx.lineWidth = 1
    ctx.strokeRect(offsetX + 0.5, offsetY + 0.5, worldW * scale, worldH * scale)

    ctx.strokeStyle = 'rgba(255,255,255,0.05)'
    ctx.beginPath()
    for (let gx = 0; gx <= worldW; gx += 5) {
      const x = Math.round(offsetX + gx * scale) + 0.5
      ctx.moveTo(x, offsetY)
      ctx.lineTo(x, offsetY + worldH * scale)
    }
    for (let gy = 0; gy <= worldH; gy += 5) {
      const y = Math.round(offsetY + gy * scale) + 0.5
      ctx.moveTo(offsetX, y)
      ctx.lineTo(offsetX + worldW * scale, y)
    }
    ctx.stroke()

    const nodeMap = new Map(topo.nodes.map(n => [n.id, n]))

    ctx.strokeStyle = 'rgba(120,180,255,0.35)'
    ctx.lineWidth = 1
    for (const e of topo.edges) {
      const src = nodeMap.get(e.source)
      const tgt = nodeMap.get(e.target)
      if (!src || !tgt) continue
      const [sx, sy] = toCanvas(src.x, src.y)
      const [tx, ty] = toCanvas(tgt.x, tgt.y)
      ctx.beginPath()
      ctx.moveTo(sx, sy)
      ctx.lineTo(tx, ty)
      ctx.stroke()
    }

    topo.nodes.forEach(node => {
      const [cx, cy] = toCanvas(node.x, node.y)

      ctx.beginPath()
      ctx.arc(cx, cy, 7, 0, Math.PI * 2)
      ctx.fillStyle = node.failed ? '#3a3a3a' : robotColor(node.id)
      ctx.fill()
      ctx.strokeStyle = '#000000'
      ctx.lineWidth = 2
      ctx.stroke()

      if (node.failed) {
        ctx.strokeStyle = '#ffffff'
        ctx.lineWidth = 1.5
        ctx.beginPath()
        ctx.moveTo(cx - 4, cy - 4); ctx.lineTo(cx + 4, cy + 4)
        ctx.moveTo(cx + 4, cy - 4); ctx.lineTo(cx - 4, cy + 4)
        ctx.stroke()
      }

      ctx.fillStyle = '#cccccc'
      ctx.font = '9px ui-monospace, monospace'
      ctx.textAlign = 'center'
      ctx.textBaseline = 'top'
      ctx.fillText(node.id.replace('robot_', 'R').toUpperCase(), cx, cy + 10)
    })

    ctx.fillStyle = '#888'
    ctx.font = '10px ui-monospace, monospace'
    ctx.textAlign = 'left'
    ctx.textBaseline = 'alphabetic'
    ctx.fillText(`COMM GRAPH · ${topo.nodes.length} NODES · ${topo.edges.length} LINKS`, 14, 18)
  }, [raw, grid])

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

  return (
    <canvas
      ref={canvasRef}
      style={{ width: '100%', height: '100%', background: '#0a0b0d', ...style }}
    />
  )
}
