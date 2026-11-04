const RETRY_MS = 1000

export type TwinPayload = string | ArrayBuffer

// the backend sends the whole output after each model run, so the last message is the state.
// null: no twin, after an empty message (new source) or a lost link (the backend may have restarted)
export function openTwinSocket(
  formatId: string,
  onPayload: (payload: TwinPayload | null) => void,
): () => void {
  let socket: WebSocket | null = null
  let retry: ReturnType<typeof setTimeout> | undefined
  let closed = false

  const connect = () => {
    const url = new URL('/ws/twin', location.href)
    url.protocol = url.protocol === 'https:' ? 'wss:' : 'ws:'
    url.searchParams.set('format', formatId)
    socket = new WebSocket(url)
    socket.binaryType = 'arraybuffer'
    socket.onmessage = (event) => onPayload(event.data === '' ? null : (event.data as TwinPayload))
    socket.onclose = () => {
      if (closed) return
      onPayload(null)
      retry = setTimeout(connect, RETRY_MS)
    }
  }

  connect()
  return () => {
    closed = true
    clearTimeout(retry)
    socket?.close()
  }
}
