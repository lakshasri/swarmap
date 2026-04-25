declare module 'roslib' {
  namespace ROSLIB {
    class Ros {
      constructor(options: { url: string })
      on(event: 'connection' | 'error' | 'close', cb: (e?: any) => void): void
      close(): void
    }
    class Topic {
      constructor(options: { ros: Ros; name: string; messageType: string })
      subscribe(cb: (msg: any) => void): void
      unsubscribe(): void
      publish(msg: Message): void
    }
    class Message {
      constructor(values: Record<string, unknown>)
    }
    class Service {
      constructor(options: { ros: Ros; name: string; serviceType: string })
      callService(
        req: ServiceRequest,
        resolve: (r: unknown) => void,
        reject: (e: unknown) => void,
      ): void
    }
    class ServiceRequest {
      constructor(values: Record<string, unknown>)
    }
  }
  export default ROSLIB
}
