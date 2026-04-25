import React, { useRef, useEffect, useCallback, useState, CSSProperties } from 'react'
import ROSLIB from 'roslib'
import { useRosTopic } from '../hooks/useRosBridge'
import { robotColor } from '../colors'

interface OccupancyGrid {
  info: {
    resolution: number
    width: number
    height: number
    origin: { position: { x: number; y: number } }
  }
  data: number[]
}

interface RobotState {
  id: string
  state: string
  battery: number
  x: number
  y: number
}

interface DashboardStats {
  robots: RobotState[]
}

type Layer = 'merged' | 'truth' | 'confidence'

interface Props {
  ros: ROSLIB.Ros | null
  style?: CSSProperties
}

const COLOR_BG   = '#0a0b0d'
const COLOR_WALL = '#ff5a3c'
const COLOR_FREE = '#e8e2d0'
const COLOR_GRID  = 'rgba(255,255,255,0.05)'
const COLOR_GRID_MAJOR = 'rgba(255,255,255,0.12)'
const COLOR_FAILED = '#454545'

function rasterizeMerged(
  grid: OccupancyGrid,
  truth: OccupancyGrid | null,
): HTMLCanvasElement {
  const { width, height } = grid.info
  const off = document.createElement('canvas')
  off.width = width
  off.height = height
  const ctx = off.getContext('2d')!
  const img = ctx.createImageData(width, height)
  const useTruth = truth?.data && truth.info.width === width && truth.info.height === height

  // First pass: classify cells
  const CELL_UNK = 0, CELL_FREE = 1, CELL_WALL = 2, CELL_TRUTH = 3
  const cells = new Uint8Array(width * height)
  for (let i = 0; i < width * height; ++i) {
    const v = grid.data[i]
    if (v === 0)        cells[i] = CELL_FREE
    else if (v === 100) cells[i] = CELL_WALL
    else if (useTruth && truth!.data[i] === 100) cells[i] = CELL_TRUTH
  }

  // Noise filter: drop isolated wall cells (0 or 1 wall neighbour are treated
  // as false positives — real walls are always continuous). Does NOT dilate.
  const filtered = new Uint8Array(cells)
  for (let gy = 1; gy < height - 1; ++gy) {
    for (let gx = 1; gx < width - 1; ++gx) {
      const i = gy * width + gx
      if (cells[i] !== CELL_WALL) continue
      let neigh = 0
      for (let dy = -1; dy <= 1; ++dy) {
        for (let dx = -1; dx <= 1; ++dx) {
          if (dx === 0 && dy === 0) continue
          if (cells[(gy + dy) * width + (gx + dx)] === CELL_WALL) ++neigh
        }
      }
      if (neigh < 2) filtered[i] = CELL_FREE   // lone wall pixel → noise
    }
  }

  const palette: Record<number, [number, number, number]> = {
    [CELL_UNK]:   [0x15, 0x17, 0x1b],
    [CELL_FREE]:  [0xe8, 0xe2, 0xd0],
    [CELL_WALL]:  [0xff, 0x44, 0x22],
    [CELL_TRUTH]: [0x3a, 0x42, 0x50],
  }
  for (let gy = 0; gy < height; ++gy) {
    for (let gx = 0; gx < width; ++gx) {
      const [r, g, b] = palette[filtered[gy * width + gx]]
      const idx = ((height - 1 - gy) * width + gx) * 4
      img.data[idx] = r; img.data[idx+1] = g; img.data[idx+2] = b; img.data[idx+3] = 255
    }
  }
  ctx.putImageData(img, 0, 0)
  return off
}

function rasterizeTruth(truth: OccupancyGrid): HTMLCanvasElement {
  const { width, height } = truth.info
  const off = document.createElement('canvas')
  off.width = width
  off.height = height
  const ctx = off.getContext('2d')!
  const img = ctx.createImageData(width, height)
  for (let gy = 0; gy < height; ++gy) {
    for (let gx = 0; gx < width; ++gx) {
      const i = gy * width + gx
      const v = truth.data[i]
      let r: number, g: number, b: number
      if (v === 100)      [r, g, b] = [0xff, 0x5a, 0x3c]
      else                [r, g, b] = [0xd8, 0xd2, 0xc0]
      const idx = ((height - 1 - gy) * width + gx) * 4
      img.data[idx] = r; img.data[idx+1] = g; img.data[idx+2] = b; img.data[idx+3] = 255
    }
  }
  ctx.putImageData(img, 0, 0)
  return off
}

function rasterizeConfidence(
  conf: OccupancyGrid,
  grid: OccupancyGrid,
): HTMLCanvasElement {
  const { width, height } = conf.info
  const off = document.createElement('canvas')
  off.width = width
  off.height = height
  const ctx = off.getContext('2d')!
  const img = ctx.createImageData(width, height)
  const gridOK = grid.info.width === width && grid.info.height === height
  for (let gy = 0; gy < height; ++gy) {
    for (let gx = 0; gx < width; ++gx) {
      const i = gy * width + gx
      const c = conf.data[i]
      const gv = gridOK ? grid.data[i] : -1
      let r: number, g: number, b: number
      if (c < 0) { [r, g, b] = [0x15, 0x17, 0x1b] }
      else if (gv === 100) { [r, g, b] = [0xff, 0x5a, 0x3c] }
      else {
        const t = Math.min(1, c / 100)
        if (t < 0.5) { const k = t / 0.5; r = 0x1e+(0x4a-0x1e)*k; g = 0x2c+(0xe3-0x2c)*k; b = 0x4a+(0xff-0x4a)*k }
        else          { const k = (t-0.5)/0.5; r = 0x4a+(0xff-0x4a)*k; g = 0xe3+(0xf2-0xe3)*k; b = 0xff+(0xa0-0xff)*k }
      }
      const idx = ((height - 1 - gy) * width + gx) * 4
      img.data[idx] = r; img.data[idx+1] = g; img.data[idx+2] = b; img.data[idx+3] = 255
    }
  }
  ctx.putImageData(img, 0, 0)
  return off
}

function drawGridOverlay(
  ctx: CanvasRenderingContext2D, ox: number, oy: number,
  scale: number, w: number, h: number,
) {
  if (scale < 2) return
  ctx.lineWidth = 1
  ctx.strokeStyle = COLOR_GRID
  const step = scale < 6 ? 5 : 1
  ctx.beginPath()
  for (let gx = 0; gx <= w; gx += step) { const x = Math.round(ox+gx*scale)+0.5; ctx.moveTo(x,oy); ctx.lineTo(x,oy+h*scale) }
  for (let gy = 0; gy <= h; gy += step) { const y = Math.round(oy+gy*scale)+0.5; ctx.moveTo(ox,y); ctx.lineTo(ox+w*scale,y) }
  ctx.stroke()
  ctx.strokeStyle = COLOR_GRID_MAJOR
  ctx.beginPath()
  for (let gx = 0; gx <= w; gx += 10) { const x = Math.round(ox+gx*scale)+0.5; ctx.moveTo(x,oy); ctx.lineTo(x,oy+h*scale) }
  for (let gy = 0; gy <= h; gy += 10) { const y = Math.round(oy+gy*scale)+0.5; ctx.moveTo(ox,y); ctx.lineTo(ox+w*scale,y) }
  ctx.stroke()
}

export default function MapCanvas({ ros, style }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const viewRef   = useRef({ offsetX: 0, offsetY: 0, scale: 4, fitted: false })
  const dragging  = useRef<{ sx: number; sy: number; ox: number; oy: number } | null>(null)
  const [layer, setLayer] = useState<Layer>('merged')

  const grid  = useRosTopic<OccupancyGrid>(ros, '/swarm/global_map',   'nav_msgs/OccupancyGrid')
  const truth = useRosTopic<OccupancyGrid>(ros, '/world/ground_truth', 'nav_msgs/OccupancyGrid')
  const conf  = useRosTopic<OccupancyGrid>(ros, '/dashboard/confidence_map', 'nav_msgs/OccupancyGrid')
  const stats = useRosTopic<{ data: string }>(ros, '/dashboard/stats', 'std_msgs/String')

  const fitToCanvas = useCallback(() => {
    const canvas = canvasRef.current
    const src = grid ?? truth
    if (!canvas || !src) return
    const { width, height } = src.info
    const sx = canvas.width / width, sy = canvas.height / height
    const scale = Math.max(0.5, Math.min(sx, sy) * 0.92)
    viewRef.current = { scale, offsetX: (canvas.width - width*scale)/2, offsetY: (canvas.height - height*scale)/2, fitted: true }
  }, [grid, truth])

  const render = useCallback(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return
    ctx.fillStyle = COLOR_BG
    ctx.fillRect(0, 0, canvas.width, canvas.height)

    const src = grid ?? truth
    if (!src) {
      ctx.fillStyle = '#5a5a5a'; ctx.font = '11px ui-monospace, monospace'; ctx.textAlign = 'center'
      ctx.fillText('WAITING FOR MAP', canvas.width/2, canvas.height/2); return
    }
    if (!viewRef.current.fitted) fitToCanvas()
    const { offsetX, offsetY, scale } = viewRef.current
    const { width, height, resolution, origin } = src.info

    let off: HTMLCanvasElement
    if (layer === 'truth' && truth?.data)        off = rasterizeTruth(truth)
    else if (layer === 'confidence' && conf?.data && grid) off = rasterizeConfidence(conf, grid)
    else                                         off = rasterizeMerged(grid!, truth)

    ctx.imageSmoothingEnabled = false
    ctx.drawImage(off, offsetX, offsetY, width*scale, height*scale)
    drawGridOverlay(ctx, offsetX, offsetY, scale, width, height)

    ctx.strokeStyle = 'rgba(255,255,255,0.25)'; ctx.lineWidth = 1
    ctx.strokeRect(offsetX+0.5, offsetY+0.5, width*scale, height*scale)

    // Dock markers — 4 docks in each quadrant
    const originX = origin.position.x, originY = origin.position.y
    const worldToCanvas = (wx: number, wy: number): [number, number] => {
      const gx = (wx - originX) / resolution
      const gy = (wy - originY) / resolution
      return [offsetX + gx*scale, offsetY + (height - gy)*scale]
    }

    const mapW = width * resolution, mapH = height * resolution
    const docks = [
      { x: originX + mapW*0.50, y: originY + mapH*0.50 },
      { x: originX + mapW*0.25, y: originY + mapH*0.25 },
      { x: originX + mapW*0.75, y: originY + mapH*0.25 },
      { x: originX + mapW*0.25, y: originY + mapH*0.75 },
      { x: originX + mapW*0.75, y: originY + mapH*0.75 },
    ]
    for (const d of docks) {
      const [dcx, dcy] = worldToCanvas(d.x, d.y)
      ctx.beginPath(); ctx.arc(dcx, dcy, 8, 0, Math.PI*2)
      ctx.fillStyle = 'rgba(0,200,100,0.2)'; ctx.fill()
      ctx.strokeStyle = '#00c864'; ctx.lineWidth = 2; ctx.stroke()
      ctx.font = '600 8px ui-monospace, monospace'; ctx.fillStyle = '#00c864'
      ctx.textAlign = 'center'; ctx.textBaseline = 'top'
      ctx.fillText('DOCK', dcx, dcy + 10)
    }

    // Robot markers
    let parsedStats: DashboardStats | null = null
    try { const d = stats?.data; if (d) parsedStats = JSON.parse(d) as DashboardStats } catch {}

    if (parsedStats) {
      for (const r of parsedStats.robots) {
        const [cx, cy] = worldToCanvas(r.x ?? 0, r.y ?? 0)
        const failed = r.state === 'FAILED'
        const col = failed ? COLOR_FAILED : robotColor(r.id)
        ctx.beginPath(); ctx.arc(cx, cy, 7, 0, Math.PI*2)
        ctx.fillStyle = col; ctx.fill()
        ctx.strokeStyle = '#000'; ctx.lineWidth = 2; ctx.stroke()
        if (failed) {
          ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.2; ctx.beginPath()
          ctx.moveTo(cx-3,cy-3); ctx.lineTo(cx+3,cy+3)
          ctx.moveTo(cx+3,cy-3); ctx.lineTo(cx-3,cy+3); ctx.stroke()
        }
        const label = r.id.replace('robot_','R').toUpperCase()
        ctx.font = '600 10px ui-monospace, monospace'
        const tw = ctx.measureText(label).width
        ctx.fillStyle = 'rgba(0,0,0,0.8)'; ctx.fillRect(cx+10, cy-8, tw+8, 16)
        ctx.fillStyle = col; ctx.textAlign = 'left'; ctx.textBaseline = 'middle'
        ctx.fillText(label, cx+14, cy)
      }
    }

    // Scale bar
    const barPx = Math.round(1.0/resolution)*scale
    ctx.fillStyle = '#fff'; ctx.fillRect(14, canvas.height-18, barPx, 2)
    ctx.font = '10px ui-monospace, monospace'; ctx.textAlign = 'left'; ctx.textBaseline = 'alphabetic'
    ctx.fillText('1 m', 14, canvas.height-22)

    // Legend
    const legend: [string, string][] = layer === 'confidence'
      ? [['#1e2c4a','LOW'],['#4ae3ff','MID'],['#fff2a0','HIGH'],[COLOR_WALL,'WALL']]
      : layer === 'truth'
      ? [[COLOR_FREE,'FREE'],[COLOR_WALL,'WALL']]
      : [[COLOR_FREE,'EXPLORED'],[COLOR_WALL,'WALL'],['#3a4250','UNKNOWN WALL'],['#15171b','UNKNOWN']]
    let lx = 14; const ly = 14
    ctx.font = '10px ui-monospace, monospace'; ctx.textBaseline = 'middle'
    for (const [c, label] of legend) {
      ctx.fillStyle = c; ctx.fillRect(lx,ly,12,12)
      ctx.strokeStyle = '#3a3a3a'; ctx.lineWidth = 1; ctx.strokeRect(lx+0.5,ly+0.5,12,12)
      ctx.fillStyle = '#ccc'; ctx.fillText(label, lx+18, ly+6)
      lx += ctx.measureText(label).width + 40
    }
  }, [grid, truth, conf, stats, fitToCanvas, layer])

  useEffect(() => { render() }, [render])
  useEffect(() => {
    const canvas = canvasRef.current; if (!canvas) return
    const obs = new ResizeObserver(() => { canvas.width = canvas.offsetWidth; canvas.height = canvas.offsetHeight; viewRef.current.fitted = false; render() })
    obs.observe(canvas); return () => obs.disconnect()
  }, [render])

  const onMouseDown = (e: React.MouseEvent) => { dragging.current = { sx: e.clientX, sy: e.clientY, ox: viewRef.current.offsetX, oy: viewRef.current.offsetY } }
  const onMouseMove = (e: React.MouseEvent) => { if (!dragging.current) return; viewRef.current.offsetX = dragging.current.ox+e.clientX-dragging.current.sx; viewRef.current.offsetY = dragging.current.oy+e.clientY-dragging.current.sy; render() }
  const onMouseUp = () => { dragging.current = null }
  const onWheel = (e: React.WheelEvent) => {
    const canvas = canvasRef.current; if (!canvas) return
    const rect = canvas.getBoundingClientRect(); const mx = e.clientX-rect.left, my = e.clientY-rect.top
    const f = e.deltaY < 0 ? 1.15 : 1/1.15; const ns = Math.max(0.5, Math.min(40, viewRef.current.scale*f))
    const k = ns / viewRef.current.scale
    viewRef.current.offsetX = mx-(mx-viewRef.current.offsetX)*k; viewRef.current.offsetY = my-(my-viewRef.current.offsetY)*k
    viewRef.current.scale = ns; render()
  }

  const layers: { id: Layer; label: string }[] = [
    { id: 'merged', label: 'MERGED' },
    { id: 'truth',  label: 'GROUND TRUTH' },
    { id: 'confidence', label: 'CONFIDENCE' },
  ]

  return (
    <div style={{ position: 'relative', width: '100%', height: '100%', background: COLOR_BG, ...style }}>
      <canvas ref={canvasRef} style={{ width:'100%', height:'100%', cursor:'grab', display:'block' }}
        onMouseDown={onMouseDown} onMouseMove={onMouseMove} onMouseUp={onMouseUp} onMouseLeave={onMouseUp} onWheel={onWheel} />
      <div style={{ position:'absolute', top:12, right:12, display:'flex', gap:0 }}>
        {layers.map(l => (
          <button key={l.id} onClick={() => setLayer(l.id)} style={{
            padding:'6px 14px', fontSize:10, letterSpacing:1.4, fontFamily:'var(--font-mono)',
            background: layer===l.id ? 'var(--text)' : 'rgba(0,0,0,0.7)',
            color: layer===l.id ? 'var(--bg)' : 'var(--text)',
            border: '1px solid var(--border-hi)',
          }}>{l.label}</button>
        ))}
        <button onClick={() => { viewRef.current.fitted = false; render() }} style={{
          padding:'6px 14px', fontSize:10, letterSpacing:1.4, fontFamily:'var(--font-mono)', background:'rgba(0,0,0,0.7)', marginLeft:8,
        }}>FIT</button>
      </div>
    </div>
  )
}
