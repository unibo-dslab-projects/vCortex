import {
  BoxGeometry,
  BufferGeometry,
  Color,
  DoubleSide,
  EdgesGeometry,
  Float32BufferAttribute,
  Group,
  LineBasicMaterial,
  LineLoop,
  LineSegments,
  Mesh,
  MeshBasicMaterial,
  Object3D,
  Shape,
  ShapeGeometry,
  Vector2,
  Vector3,
} from 'three'
import type { Level, Twin } from '@/types/twin'

const ACCENT = new Color('#22d3ee')
const FILLED_IN = new Color('#fb923c')
const OBJECT = new Color('#34d399')
const WALL_FILL = new Color('#17323d')

export interface TwinLook {
  wireframe: boolean
  coverage: boolean
}

export interface Label {
  key: string
  // map frame, z up
  at: [number, number, number]
  text: string
  kind: 'dimension' | 'object'
}

export interface BuiltTwin {
  group: Group
  labels: Label[]
  min: Vector3
  max: Vector3
  floorZ: number
}

const fmt = (metres: number) => metres.toFixed(2)

function grow(out: BuiltTwin, x: number, y: number, z: number) {
  out.min.min(new Vector3(x, y, z))
  out.max.max(new Vector3(x, y, z))
}

function outlined(geometry: BufferGeometry, edge: Color, fill: Color, fillOpacity: number) {
  const holder = new Object3D()
  if (fillOpacity > 0) {
    holder.add(
      new Mesh(
        geometry,
        new MeshBasicMaterial({
          color: fill,
          transparent: true,
          opacity: fillOpacity,
          depthWrite: false,
          side: DoubleSide,
        }),
      ),
    )
  }
  holder.add(new LineSegments(new EdgesGeometry(geometry), new LineBasicMaterial({ color: edge })))
  return holder
}

function addWalls(level: Level, look: TwinLook, out: BuiltTwin) {
  const height = level.wall_height
  for (const wall of level.walls) {
    const [x0, y0] = wall.start
    const [x1, y1] = wall.end
    const length = Math.hypot(x1 - x0, y1 - y0)
    // coverage 1 = the whole length was seen; the rest was bridged by the model
    const missing = 1 - wall.coverage
    const edge = look.coverage ? ACCENT.clone().lerp(FILLED_IN, Math.min(1, missing * 2)) : ACCENT
    const fill = look.coverage ? WALL_FILL.clone().lerp(FILLED_IN, missing) : WALL_FILL
    const box = outlined(
      new BoxGeometry(length, Math.max(wall.thickness, 0.02), height),
      edge,
      fill,
      look.wireframe ? 0 : 0.55,
    )
    box.position.set((x0 + x1) / 2, (y0 + y1) / 2, level.floor_z + height / 2)
    box.rotation.z = Math.atan2(y1 - y0, x1 - x0)
    out.group.add(box)

    out.labels.push({
      key: `${wall.id}-length`,
      at: [(x0 + x1) / 2, (y0 + y1) / 2, level.floor_z + height + 0.05],
      text: look.coverage
        ? `${wall.id} · ${Math.round(wall.coverage * 100)}% seen`
        : `L ${fmt(length)} m`,
      kind: 'dimension',
    })
    grow(out, x0, y0, level.floor_z)
    grow(out, x1, y1, level.floor_z + height)
  }

  // one height marker on the longest wall: every wall of a level shares its height
  const longest = [...level.walls].sort(
    (a, b) =>
      Math.hypot(b.end[0] - b.start[0], b.end[1] - b.start[1]) -
      Math.hypot(a.end[0] - a.start[0], a.end[1] - a.start[1]),
  )[0]
  if (longest) {
    const [x, y] = longest.start
    const tick = new BufferGeometry()
    tick.setAttribute(
      'position',
      new Float32BufferAttribute([x, y, level.floor_z, x, y, level.floor_z + height], 3),
    )
    out.group.add(new LineSegments(tick, new LineBasicMaterial({ color: 0xffffff })))
    out.labels.push({
      key: `${level.id}-height`,
      at: [x, y, level.floor_z + height / 2],
      text: `H ${fmt(height)} m${level.wall_height_source === 'default' ? ' (default)' : ''}`,
      kind: 'dimension',
    })
  }
}

function addRooms(level: Level, out: BuiltTwin) {
  for (const room of level.rooms) {
    const colour = room.closed ? ACCENT : FILLED_IN
    const floor = new Mesh(
      new ShapeGeometry(new Shape(room.outline.map(([x, y]) => new Vector2(x, y)))),
      new MeshBasicMaterial({
        color: colour,
        transparent: true,
        opacity: 0.07,
        depthWrite: false,
        side: DoubleSide,
      }),
    )
    floor.position.z = level.floor_z + 0.005
    const ring = new BufferGeometry().setFromPoints(
      room.outline.map(([x, y]) => new Vector3(x, y, level.floor_z + 0.01)),
    )
    out.group.add(floor, new LineLoop(ring, new LineBasicMaterial({ color: colour })))
  }
}

function addObjects(level: Level, out: BuiltTwin) {
  for (const object of level.objects) {
    const height = Math.max(object.top_z_seen - object.bottom_z, 0.02)
    const box = outlined(
      new BoxGeometry(object.size[0], object.size[1], height),
      OBJECT,
      OBJECT,
      0.1,
    )
    box.position.set(object.center[0], object.center[1], object.bottom_z + height / 2)
    box.rotation.z = object.yaw
    out.group.add(box)
    out.labels.push({
      key: object.id,
      at: [object.center[0], object.center[1], object.top_z_seen + 0.05],
      text: `${object.id} [${fmt(object.size[0])}×${fmt(object.size[1])}×${fmt(height)} m]`,
      kind: 'object',
    })
    grow(out, object.center[0], object.center[1], object.bottom_z)
  }
}

export function buildTwin(twin: Twin, look: TwinLook): BuiltTwin {
  const out: BuiltTwin = {
    group: new Group(),
    labels: [],
    min: new Vector3(Infinity, Infinity, Infinity),
    max: new Vector3(-Infinity, -Infinity, -Infinity),
    floorZ: twin.levels[0].floor_z,
  }
  for (const level of twin.levels) {
    addRooms(level, out)
    addWalls(level, look, out)
    addObjects(level, out)
  }
  return out
}

export function disposeTree(root: Object3D) {
  root.traverse((node) => {
    const drawn = node as Partial<Mesh>
    drawn.geometry?.dispose()
    const material = drawn.material
    if (Array.isArray(material)) material.forEach((m) => m.dispose())
    else material?.dispose()
  })
}
