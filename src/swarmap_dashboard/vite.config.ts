import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],

  server: {
    port: 5173,
    proxy: {
      // Proxy WebSocket connections to rosbridge during development
      '/rosbridge': {
        target: 'ws://localhost:9090',
        ws: true,
        rewrite: (path) => path.replace(/^\/rosbridge/, ''),
      },
    },
  },

  build: {
    outDir: 'dist',
    sourcemap: true,
  },

  preview: {
    port: 5173,
    host: '0.0.0.0',
  },
})
