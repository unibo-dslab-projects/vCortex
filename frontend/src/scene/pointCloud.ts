import {
  BufferAttribute,
  BufferGeometry,
  Color,
  Points,
  PointsMaterial,
  Vector3,
  type ColorRepresentation,
} from 'three'

const LOW = new Color('#0b3a46')
const MID = new Color('#22d3ee')
const HIGH = new Color('#e0fbff')

// map frame points (x y z, z up) coloured by height. deltas write in place: rebuilding an
// 800k-point geometry per delta stalls the page
export class PointCloud {
  readonly object: Points<BufferGeometry, PointsMaterial>
  readonly min = new Vector3(Infinity, Infinity, Infinity)
  readonly max = new Vector3(-Infinity, -Infinity, -Infinity)
  count = 0
  private positions = new Float32Array(0)
  private colors = new Float32Array(0)
  // 2nd height percentile: close to the floor, the views put their grid there
  zLow = 0
  private zHigh = 1

  constructor(size = 0.03, tint?: ColorRepresentation) {
    const material = new PointsMaterial({
      size,
      sizeAttenuation: true,
      vertexColors: !tint,
      color: tint ?? 0xffffff,
      transparent: true,
      opacity: 0.9,
      depthWrite: false,
    })
    this.object = new Points(new BufferGeometry(), material)
    this.object.frustumCulled = false
  }

  reset(points: Float32Array) {
    this.count = 0
    this.min.set(Infinity, Infinity, Infinity)
    this.max.set(-Infinity, -Infinity, -Infinity)
    this.pickHeightRange(points)
    this.append(points)
  }

  private append(points: Float32Array) {
    const added = points.length / 3
    this.reserve(this.count + added)
    const color = new Color()
    for (let i = 0; i < added; i++) {
      this.place(this.count + i, points[i * 3]!, points[i * 3 + 1]!, points[i * 3 + 2]!, color)
    }
    this.count += added
    this.changed()
  }

  // delta records, 4 values each: x y z from records, the voxel's index from indices[3].
  // the backend updates voxels it already sent, so an index below count replaces that point
  upsert(records: Float32Array, indices: Uint32Array) {
    const n = records.length / 4
    let end = this.count
    for (let i = 0; i < n; i++) end = Math.max(end, indices[i * 4 + 3]! + 1)
    this.reserve(end)
    const color = new Color()
    for (let i = 0; i < n; i++) {
      const at = indices[i * 4 + 3]!
      this.place(at, records[i * 4]!, records[i * 4 + 1]!, records[i * 4 + 2]!, color)
    }
    this.count = end
    this.changed()
  }

  dispose() {
    this.object.geometry.dispose()
    this.object.material.dispose()
  }

  private place(at: number, x: number, y: number, z: number, color: Color) {
    this.positions[at * 3] = x
    this.positions[at * 3 + 1] = y
    this.positions[at * 3 + 2] = z
    this.min.set(Math.min(this.min.x, x), Math.min(this.min.y, y), Math.min(this.min.z, z))
    this.max.set(Math.max(this.max.x, x), Math.max(this.max.y, y), Math.max(this.max.z, z))
    this.heightColor(z, color).toArray(this.colors, at * 3)
  }

  private changed() {
    const geometry = this.object.geometry
    geometry.setDrawRange(0, this.count)
    geometry.attributes.position!.needsUpdate = true
    geometry.attributes.color!.needsUpdate = true
  }

  // 2nd and 98th percentile: a few stray returns must not wash out the whole gradient
  private pickHeightRange(points: Float32Array) {
    const n = points.length / 3
    if (n === 0) return
    const step = Math.max(1, Math.floor(n / 20000))
    const sample: number[] = []
    for (let i = 0; i < n; i += step) sample.push(points[i * 3 + 2]!)
    sample.sort((a, b) => a - b)
    this.zLow = sample[Math.floor(sample.length * 0.02)]!
    this.zHigh = sample[Math.floor(sample.length * 0.98)]!
    if (this.zHigh - this.zLow < 0.5) this.zHigh = this.zLow + 0.5
  }

  private heightColor(z: number, out: Color): Color {
    const t = Math.min(1, Math.max(0, (z - this.zLow) / (this.zHigh - this.zLow)))
    return t < 0.6 ? out.lerpColors(LOW, MID, t / 0.6) : out.lerpColors(MID, HIGH, (t - 0.6) / 0.4)
  }

  // doubling keeps the copies rare when a live map grows by small deltas
  private reserve(needed: number) {
    if (needed * 3 <= this.positions.length) return
    let capacity = Math.max(1024, this.positions.length / 3)
    while (capacity < needed) capacity *= 2
    const positions = new Float32Array(capacity * 3)
    const colors = new Float32Array(capacity * 3)
    positions.set(this.positions.subarray(0, this.count * 3))
    colors.set(this.colors.subarray(0, this.count * 3))
    this.positions = positions
    this.colors = colors
    // a larger buffer needs a new attribute: webgl cannot resize one in place
    const geometry = this.object.geometry
    geometry.dispose()
    geometry.setAttribute('position', new BufferAttribute(positions, 3))
    geometry.setAttribute('color', new BufferAttribute(colors, 3))
  }
}
