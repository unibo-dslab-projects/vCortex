import { Box3, Vector3, type Camera } from 'three'

// the backend speaks the map frame (z up, REP-103); three.js is y up. one turn of the whole
// scene keeps every coordinate as the backend sent it
export const Z_UP_TO_Y_UP: [number, number, number] = [-Math.PI / 2, 0, 0]

export type ViewMode = 'iso' | 'top'

// the part of cientos' OrbitControls we use; its class lives in three-stdlib, not a direct dep
export interface Orbit {
  object: Camera
  target: Vector3
  update(): void
}

// map frame (x, y, z up) to three.js (x, z, -y)
export function mapBoxToThree(min: Vector3, max: Vector3): Box3 {
  return new Box3(new Vector3(min.x, min.z, -max.y), new Vector3(max.x, max.z, -min.y))
}

export function frameView(controls: Orbit | null | undefined, box: Box3, mode: ViewMode) {
  if (!controls || box.isEmpty()) return
  const center = box.getCenter(new Vector3())
  const size = box.getSize(new Vector3())
  const reach = Math.max(size.x, size.z, 2)
  if (mode === 'top') {
    // a tiny z offset: straight down makes OrbitControls lose its up direction
    controls.object.position.set(center.x, center.y + reach * 1.3, center.z + 0.001)
  } else {
    controls.object.position.set(
      center.x + reach * 0.85,
      center.y + reach * 0.9,
      center.z + reach * 0.85,
    )
  }
  controls.target.copy(center)
  controls.update()
}
