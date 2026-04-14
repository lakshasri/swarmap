import { useEffect, useRef, useState, useCallback } from 'react'

import ROSLIB from 'roslib'

const WS_URL = 'ws://localhost:9090'

export interface RosBridgeState {
  ros: ROSLIB.Ros | null
  connected: boolean
  error: string | null
}

export function useRosBridge(): RosBridgeState {
  const [connected, setConnected] = useState(false)
  const [error, setError]         = useState<string | null>(null)
  const rosRef = useRef<ROSLIB.Ros | null>(null)

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: WS_URL })
    rosRef.current = ros

    ros.on('connection', () => {
      setConnected(true)
      setError(null)
    })
    ros.on('error', (e: Error) => {
      setError(e.message ?? 'rosbridge error')
    })
    ros.on('close', () => {
      setConnected(false)
    })

    return () => {
      ros.close()
    }
  }, [])

  return { ros: rosRef.current, connected, error }
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
