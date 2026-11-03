<script setup lang="ts">
import { Html } from '@tresjs/cientos'
import { computed, onBeforeUnmount, shallowRef, watch } from 'vue'
import type { Vector3 } from 'three'
import { buildTwin, disposeTree, type BuiltTwin, type TwinLook } from '@/scene/twinScene'
import type { Twin } from '@/types/twin'

const props = defineProps<{
  payload: string
  look: TwinLook
  dimensions: boolean
  objectLabels: boolean
}>()
const emit = defineEmits<{ bounds: [min: Vector3, max: Vector3, floorZ: number] }>()

const twin = computed(() => JSON.parse(props.payload) as Twin)
const built = shallowRef<BuiltTwin | null>(null)

watch(
  [twin, () => props.look.wireframe, () => props.look.coverage],
  ([next], [previous]) => {
    if (built.value) disposeTree(built.value.group)
    built.value = buildTwin(next, props.look)
    if (next !== previous) emit('bounds', built.value.min, built.value.max, built.value.floorZ)
  },
  { immediate: true },
)

const labels = computed(
  () =>
    built.value?.labels.filter((label) =>
      label.kind === 'object' ? props.objectLabels : props.dimensions,
    ) ?? [],
)

onBeforeUnmount(() => {
  if (built.value) disposeTree(built.value.group)
})
</script>

<template>
  <template v-if="built">
    <primitive :object="built.group" />
    <Html v-for="label in labels" :key="label.key" :position="label.at" center>
      <span class="label-3d" :class="label.kind">{{ label.text }}</span>
    </Html>
  </template>
</template>
