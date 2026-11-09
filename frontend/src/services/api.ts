export interface Format {
  id: string
  label: string
  binary: boolean
}

export interface Settings {
  wall_height: number
  wall_height_source: 'default' | 'user'
  walls: { min_top_ratio: number }
}

export type SettingsPatch = { wall_height?: number; walls?: { min_top_ratio?: number } }

export interface ModelReport {
  points_in: number
  after_outliers: number
  tilt_deg: number
  yaw_deg: number
  poses: number
  candidates: number
  walls: number
  rooms: number
  objects: number
  shell_top_seen_m: number
  duration_ms: number
}

export interface Status {
  mode: 'live' | 'offline'
  source: string
  connected: boolean
  run_id: number | null
  seq: number
  points: number
  model: ModelReport | null
  clients: { raw: number; twin: number }
}

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  const res = await fetch(path, init)
  if (!res.ok) {
    const body = (await res.json().catch(() => null)) as { error?: string } | null
    throw new Error(body?.error ?? `${res.status} ${res.statusText}`)
  }
  return (await res.json()) as T
}

export const getFormats = () => request<Format[]>('/api/formats')
export const getSettings = () => request<Settings>('/api/settings')
export const getStatus = () => request<Status>('/api/status')

export const putSettings = (patch: SettingsPatch) =>
  request<Settings>('/api/settings', {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(patch),
  })

export const downloadUrl = (formatId: string) => `/api/twin/${encodeURIComponent(formatId)}`
