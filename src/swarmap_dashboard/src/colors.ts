export const ROBOT_PALETTE = [
  '#00d4ff',
  '#ff4dd2',
  '#ffd84d',
  '#4dff88',
  '#ff944d',
  '#b86bff',
  '#ff5050',
  '#d4ff4d',
  '#ff6b9d',
  '#4de6e6',
  '#ffaa00',
  '#7fff00',
]

export function robotColor(id: string): string {
  const m = id.match(/(\d+)$/)
  const idx = m ? parseInt(m[1], 10) : 0
  return ROBOT_PALETTE[idx % ROBOT_PALETTE.length]
}

export const MAP_FREE  = '#f0e9d8'
export const MAP_OCC   = '#0a0a0a'
