import React, { useRef, useEffect, useCallback, CSSProperties } from 'react'
import ROSLIB from 'roslib'
import { useRosTopic } from '../hooks/useRosBridge'

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

interface Props {
  ros: ROSLIB.Ros | null
  style?: CSSProperties
}

const ROBOT_COLORS = [
  '#4a9eff','#f5a623','#4caf7d','#e05252',
  '#b47aff','#ff6b9d','#00d4aa','#ffdd57',
  '#ff8c42','#6ef4d4',
]

export default function NetworkGraph({ ros, style }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const raw = useRosTopic<{ data: string }>(ros, '/dashboard/network_topology', 'std_msgs/String')

  const render = useCallback(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return

    ctx.clearRect(0, 0, canvas.width, canvas.height)

    let topo: Topology | null = null
    try {
      if (raw?.data) topo = JSON.parse(raw.data) as Topology
    } catch { return }
    if (!topo || topo.nodes.length === 0) {
      ctx.fillStyle = 'var(--text-muted)'
      ctx.font = '12px sans-serif'
      ctx.textAlign = 'center'
      ctx.fillText('Waiting for topology…', canvas.width / 2, canvas.height / 2)
      return
    }

    // Compute bounding box to fit canvas
    const xs = topo.nodes.map(n => n.x)
    const ys = topo.nodes.map(n => n.y)
    const minX = Math.min(...xs), maxX = Math.max(...xs)
    const minY = Math.min(...ys), maxY = Math.max(...ys)
    const pad = 30
    const rangeX = maxX - minX || 1
    const rangeY = maxY - minY || 1

    const toCanvas = (wx: number, wy: number): [number, number] => [
      pad + ((wx - minX) / rangeX) * (canvas.width  - 2 * pad),
      pad + ((maxY - wy) / rangeY) * (canvas.height - 2 * pad),
    ]

    const nodeMap = new Map(topo.nodes.map(n => [n.id, n]))

    // Draw edges
    for (const e of topo.edges) {
      const src = nodeMap.get(e.source)
      const tgt = nodeMap.get(e.target)
      if (!src || !tgt) continue

      const [sx, sy] = toCanvas(src.x, src.y)
      const [tx, ty] = toCanvas(tgt.x, tgt.y)

      ctx.beginPath()
      ctx.moveTo(sx, sy)
      ctx.lineTo(tx, ty)
      ctx.strokeStyle = 'rgba(74,158,255,0.35)'
      ctx.lineWidth = 1
      ctx.stroke()
    }

    // Draw nodes
    topo.nodes.forEach((node, i) => {
      const [cx, cy] = toCanvas(node.x, node.y)
      const color = ROBOT_COLORS[i % ROBOT_COLORS.length]

      ctx.beginPath()
      ctx.arc(cx, cy, 7, 0, Math.PI * 2)
      ctx.fillStyle = node.failed ? '#444' : color
      ctx.fill()

      if (node.failed) {
        // ✕ overlay
        ctx.strokeStyle = '#e05252'
        ctx.lineWidth = 2
        ctx.beginPath()
        ctx.moveTo(cx - 5, cy - 5); ctx.lineTo(cx + 5, cy + 5)
        ctx.moveTo(cx + 5, cy - 5); ctx.lineTo(cx - 5, cy + 5)
        ctx.stroke()
      }

      // ID label
      ctx.fillStyle = 'var(--text-primary)'
      ctx.font = '9px monospace'
      ctx.textAlign = 'center'
      ctx.fillText(node.id.replace('robot_', 'R'), cx, cy + 18)
    })
  }, [raw])

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
      style={{ width: '100%', height: '100%', background: 'var(--bg-secondary)', ...style }}
    />
  )
}
