<script setup lang="ts">
import { Html } from '@tresjs/cientos'
import DxfParser, { type ILineEntity, type ILwpolylineEntity, type ITextEntity } from 'dxf-parser'
import { BufferGeometry, Group, LineBasicMaterial, LineSegments, Vector3 } from 'three'
import { computed, onBeforeUnmount, shallowRef, watch } from 'vue'
import { disposeTree } from '@/scene/twinScene'

const props = defineProps<{ payload: string; labels: boolean }>()
const emit = defineEmits<{ bounds: [min: Vector3, max: Vector3, floorZ: number] }>()

const LAYER_COLOURS: Record<string, string> = {
  WALLS: '#22d3ee',
  ROOMS: '#0e7490',
  OBJECTS: '#34d399',
}

interface Built {
  group: Group
  texts: { key: string; at: [number, number, number]; text: string }[]
}

const built = shallowRef<Built | null>(null)

function build(source: string): Built {
  const dxf = new DxfParser().parseSync(source)
  const segments = new Map<string, Vector3[]>()
  const texts: Built['texts'] = []
  const min = new Vector3(Infinity, Infinity, 0)
  const max = new Vector3(-Infinity, -Infinity, 0)
  const push = (layer: string, a: { x: number; y: number }, b: { x: number; y: number }) => {
    const list = segments.get(layer) ?? []
    list.push(new Vector3(a.x, a.y, 0), new Vector3(b.x, b.y, 0))
    segments.set(layer, list)
    min.min(new Vector3(Math.min(a.x, b.x), Math.min(a.y, b.y), 0))
    max.max(new Vector3(Math.max(a.x, b.x), Math.max(a.y, b.y), 0))
  }

  for (const entity of dxf?.entities ?? []) {
    if (entity.type === 'LINE') {
      const [a, b] = (entity as ILineEntity).vertices
      if (a && b) push(entity.layer, a, b)
    } else if (entity.type === 'LWPOLYLINE') {
      const polyline = entity as ILwpolylineEntity
      const v = polyline.vertices
      for (let i = 0; i + 1 < v.length; i++) push(entity.layer, v[i]!, v[i + 1]!)
      if (polyline.shape && v.length > 2) push(entity.layer, v[v.length - 1]!, v[0]!)
    } else if (entity.type === 'TEXT') {
      const text = entity as ITextEntity
      texts.push({
        key: `${texts.length}`,
        at: [text.startPoint.x, text.startPoint.y, 0.02],
        text: text.text,
      })
    }
  }

  const group = new Group()
  for (const [layer, points] of segments) {
    group.add(
      new LineSegments(
        new BufferGeometry().setFromPoints(points),
        new LineBasicMaterial({ color: LAYER_COLOURS[layer] ?? '#7d8f9c' }),
      ),
    )
  }
  emit('bounds', min, max, 0)
  return { group, texts }
}

watch(
  () => props.payload,
  (payload) => {
    if (built.value) disposeTree(built.value.group)
    built.value = build(payload)
  },
  { immediate: true },
)

const texts = computed(() => (props.labels ? (built.value?.texts ?? []) : []))

onBeforeUnmount(() => {
  if (built.value) disposeTree(built.value.group)
})
</script>

<template>
  <template v-if="built">
    <primitive :object="built.group" />
    <Html v-for="text in texts" :key="text.key" :position="text.at" center>
      <span class="label-3d">{{ text.text }}</span>
    </Html>
  </template>
</template>
