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

// map frame points (x y z, z up) coloured by height. deltas append in place: rebuilding an
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

  append(points: Float32Array) {
    const added = points.length / 3
    this.reserve(this.count + added)
    this.positions.set(points, this.count * 3)
    const color = new Color()
    for (let i = 0; i < added; i++) {
      const x = points[i * 3]!
      const y = points[i * 3 + 1]!
      const z = points[i * 3 + 2]!
      this.min.set(Math.min(this.min.x, x), Math.min(this.min.y, y), Math.min(this.min.z, z))
      this.max.set(Math.max(this.max.x, x), Math.max(this.max.y, y), Math.max(this.max.z, z))
      this.heightColor(z, color).toArray(this.colors, (this.count + i) * 3)
    }
    this.count += added
    const geometry = this.object.geometry
    geometry.setDrawRange(0, this.count)
    geometry.attributes.position!.needsUpdate = true
    geometry.attributes.color!.needsUpdate = true
  }

  dispose() {
    this.object.geometry.dispose()
    this.object.material.dispose()
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
