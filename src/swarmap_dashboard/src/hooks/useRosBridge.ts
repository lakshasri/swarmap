import { useEffect, useRef, useState } from 'react'

import ROSLIB from 'roslib'

const WS_URL = 'ws://localhost:9090'
const RECONNECT_DELAY_MS = 1500

export interface RosBridgeState {
  ros: ROSLIB.Ros | null
  connected: boolean
  error: string | null
}

export function useRosBridge(): RosBridgeState {
  const [connected, setConnected] = useState(false)
  const [error, setError]         = useState<string | null>(null)
  const [ros, setRos]             = useState<ROSLIB.Ros | null>(null)
  const cancelledRef               = useRef(false)

  useEffect(() => {
    cancelledRef.current = false
    let reconnectTimer: ReturnType<typeof setTimeout> | null = null

    const connect = () => {
      if (cancelledRef.current) return
      const instance = new ROSLIB.Ros({ url: WS_URL })
      setRos(instance)

      instance.on('connection', () => {
        setConnected(true)
        setError(null)
      })
      instance.on('error', (e: Error) => {
        setError(e.message ?? 'rosbridge error')
      })
      instance.on('close', () => {
        setConnected(false)
        if (!cancelledRef.current) {
          reconnectTimer = setTimeout(connect, RECONNECT_DELAY_MS)
        }
      })
    }

    connect()

    return () => {
      cancelledRef.current = true
      if (reconnectTimer) clearTimeout(reconnectTimer)
      if (ros) ros.close()
    }
  }, [])

  return { ros, connected, error }
}

export function useRosTopic<T>(
  ros: ROSLIB.Ros | null,
  name: string,
  messageType: string
): T | null {
  const [msg, setMsg] = useState<T | null>(null)

  useEffect(() => {
    if (!ros) return

    const topic = new ROSLIB.Topic({ ros, name, messageType })
    topic.subscribe((m: unknown) => setMsg(m as T))

    return () => {
      topic.unsubscribe()
    }
  }, [ros, name, messageType])

  return msg
}

export function callRosService<Req, Res>(
  ros: ROSLIB.Ros,
  name: string,
  serviceType: string,
  request: Req
): Promise<Res> {
  return new Promise((resolve, reject) => {
    const svc = new ROSLIB.Service({ ros, name, serviceType })
    const req = new ROSLIB.ServiceRequest(request as Record<string, unknown>)
    svc.callService(req, resolve as (r: unknown) => void, reject)
  })
}
