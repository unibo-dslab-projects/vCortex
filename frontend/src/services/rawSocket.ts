// /ws/raw frame: kind u8, 3 zero bytes, run_id u32, seq u32, count u32 (little endian),
// then count records of x y z float32 in the map frame, z up
const HEADER_BYTES = 16
const KIND_MAP = 1
const KIND_DELTA = 2
const RETRY_MS = 1000

export interface RawHandlers {
  full(points: Float32Array, runId: number, seq: number): void
  delta(points: Float32Array, seq: number): void
  connection(open: boolean): void
}

export function openRawSocket(handlers: RawHandlers): () => void {
  let socket: WebSocket | null = null
  let retry: ReturnType<typeof setTimeout> | undefined
  let closed = false

  const connect = () => {
    const url = new URL('/ws/raw', location.href)
    url.protocol = url.protocol === 'https:' ? 'wss:' : 'ws:'
    socket = new WebSocket(url)
    socket.binaryType = 'arraybuffer'
    socket.onopen = () => handlers.connection(true)
    socket.onmessage = (event) => {
      if (!(event.data instanceof ArrayBuffer) || event.data.byteLength < HEADER_BYTES) return
      const view = new DataView(event.data)
      const kind = view.getUint8(0)
      const runId = view.getUint32(4, true)
      const seq = view.getUint32(8, true)
      const count = view.getUint32(12, true)
      const points = new Float32Array(event.data, HEADER_BYTES, count * 3)
      if (kind === KIND_MAP) handlers.full(points, runId, seq)
      else if (kind === KIND_DELTA) handlers.delta(points, seq)
    }
    socket.onclose = () => {
      handlers.connection(false)
      if (!closed) retry = setTimeout(connect, RETRY_MS)
    }
  }

  connect()
  return () => {
    closed = true
    clearTimeout(retry)
    socket?.close()
  }
}
