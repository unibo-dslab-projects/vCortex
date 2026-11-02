<script setup lang="ts">
import { PLYLoader } from 'three/addons/loaders/PLYLoader.js'
import type { Vector3 } from 'three'
import { onBeforeUnmount, watch } from 'vue'
import { PointCloud } from '@/scene/pointCloud'

const props = defineProps<{ payload: ArrayBuffer }>()
const emit = defineEmits<{ bounds: [min: Vector3, max: Vector3, floorZ: number] }>()

const cloud = new PointCloud(0.03)

watch(
  () => props.payload,
  (payload) => {
    const geometry = new PLYLoader().parse(payload)
    cloud.reset(geometry.getAttribute('position').array as Float32Array)
    geometry.dispose()
    emit('bounds', cloud.min, cloud.max, cloud.zLow)
  },
  { immediate: true },
)

onBeforeUnmount(() => cloud.dispose())
</script>

<template>
  <primitive :object="cloud.object" />
</template>
