<script setup lang="ts">
import { TresCanvas } from '@tresjs/core'
import { Grid, OrbitControls } from '@tresjs/cientos'
import { DoubleSide, type Box3 } from 'three'
import { shallowRef, watch } from 'vue'
import { frameView, Z_UP_TO_Y_UP, type Orbit, type ViewMode } from '@/scene/view'

withDefaults(defineProps<{ floorZ?: number }>(), { floorZ: 0 })

const controls = shallowRef<{ instance: Orbit | null } | null>(null)
// the first frame can arrive before the controls exist
const pending = shallowRef<[Box3, ViewMode] | null>(null)

watch(
  () => controls.value?.instance,
  (instance) => {
    if (instance && pending.value) frameView(instance, ...pending.value)
    pending.value = null
  },
)

defineExpose({
  frame(box: Box3, mode: ViewMode) {
    const instance = controls.value?.instance
    if (instance) frameView(instance, box, mode)
    else pending.value = [box, mode]
  },
})
</script>

<template>
  <div class="canvas">
    <TresCanvas clear-color="#0b1016" :antialias="true">
      <TresPerspectiveCamera :position="[8, 8, 8]" :fov="45" :near="0.05" :far="500" />
      <OrbitControls ref="controls" make-default :enable-damping="true" />
      <TresAmbientLight :intensity="0.7" />
      <TresDirectionalLight :position="[6, 12, 4]" :intensity="1.1" />
      <!-- the grid is drawn in three.js space, so the floor height z becomes y -->
      <Grid
        :args="[400, 400]"
        :position="[0, floorZ - 0.01, 0]"
        cell-color="#132a33"
        section-color="#1c4e5c"
        :cell-size="0.5"
        :section-size="1"
        :cell-thickness="0.6"
        :section-thickness="1"
        :fade-distance="70"
        :side="DoubleSide"
        infinite-grid
      />
      <TresGroup :rotation="Z_UP_TO_Y_UP">
        <slot />
      </TresGroup>
    </TresCanvas>
  </div>
</template>

<style scoped>
/* out of the flow: a canvas in a flex child keeps its first height when the panel grows */
.canvas {
  position: absolute;
  inset: 0;
  overflow: hidden;
}
</style>
