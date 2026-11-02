<script setup lang="ts">
import { computed, onBeforeUnmount, reactive, ref, shallowRef, useTemplateRef, watch } from 'vue'
import type { Vector3 } from 'three'
import SceneCanvas from '@/components/SceneCanvas.vue'
import ViewTools from '@/components/ViewTools.vue'
import DxfScene from '@/components/renderers/DxfScene.vue'
import PlyScene from '@/components/renderers/PlyScene.vue'
import TwinJsonScene from '@/components/renderers/TwinJsonScene.vue'
import type { Format } from '@/services/api'
import { openTwinSocket, type TwinPayload } from '@/services/twinSocket'
import { mapBoxToThree, type ViewMode } from '@/scene/view'

const props = defineProps<{ formats: Format[] }>()
const selected = defineModel<string>({ required: true })

const scene = useTemplateRef('scene')
const payload = shallowRef<TwinPayload | null>(null)
const floorZ = ref(0)
const mode = ref<ViewMode>('iso')
const look = reactive({ wireframe: false, coverage: false })
const dimensions = ref(true)
const objectLabels = ref(false)

let bounds: [Vector3, Vector3] | null = null
let framed = false

const view = (next: ViewMode) => {
  mode.value = next
  if (bounds) scene.value?.frame(mapBoxToThree(...bounds), next)
}

const onBounds = (min: Vector3, max: Vector3, floor: number) => {
  bounds = [min.clone(), max.clone()]
  floorZ.value = floor
  // only the first output of a format moves the camera: later runs must not undo the user's view
  if (!framed) view(mode.value)
  framed = true
}

let close = () => {}
watch(
  selected,
  (id) => {
    close()
    payload.value = null
    bounds = null
    framed = false
    // a floor plan reads best from above
    mode.value = id === 'dxf' ? 'top' : 'iso'
    close = openTwinSocket(id, (data) => (payload.value = data))
  },
  { immediate: true },
)
onBeforeUnmount(() => close())

const text = computed(() => (typeof payload.value === 'string' ? payload.value : null))
const binary = computed(() => (payload.value instanceof ArrayBuffer ? payload.value : null))
const current = computed(() => props.formats.find((f) => f.id === selected.value))
</script>

<template>
  <section class="panel">
    <header>
      <span class="panel-title"><span class="dot" :class="{ on: payload }" />Digital twin</span>
      <nav class="tabs" aria-label="Output format">
        <button
          v-for="format in formats"
          :key="format.id"
          class="chip"
          :class="{ active: format.id === selected }"
          :aria-pressed="format.id === selected"
          @click="selected = format.id"
        >
          {{ format.label }}
        </button>
      </nav>
    </header>

    <div class="stage">
      <SceneCanvas ref="scene" :floor-z="floorZ">
        <TwinJsonScene
          v-if="selected === 'twin-json' && text"
          :payload="text"
          :look="look"
          :dimensions="dimensions"
          :object-labels="objectLabels"
          @bounds="onBounds"
        />
        <DxfScene
          v-else-if="selected === 'dxf' && text"
          :payload="text"
          :labels="dimensions"
          @bounds="onBounds"
        />
        <PlyScene v-else-if="selected === 'ply' && binary" :payload="binary" @bounds="onBounds" />
      </SceneCanvas>

      <ViewTools :mode="mode" @view="view" />

      <p v-if="!payload" class="empty">
        Waiting for the first model run of {{ current?.label ?? selected }}…
      </p>

      <div class="scale hint"><i />grid 0.5 m · line 1 m</div>

      <div v-if="selected !== 'ply'" class="look" role="toolbar" aria-label="Display">
        <template v-if="selected === 'twin-json'">
          <button
            class="chip"
            :class="{ active: look.wireframe }"
            :aria-pressed="look.wireframe"
            @click="look.wireframe = !look.wireframe"
          >
            Wireframe
          </button>
          <button
            class="chip"
            :class="{ active: look.coverage }"
            :aria-pressed="look.coverage"
            title="Orange where the model filled in a wall it did not see"
            @click="look.coverage = !look.coverage"
          >
            Coverage
          </button>
        </template>
        <button
          class="chip"
          :class="{ active: dimensions }"
          :aria-pressed="dimensions"
          @click="dimensions = !dimensions"
        >
          {{ selected === 'dxf' ? 'Labels' : 'Dimensions' }}
        </button>
        <button
          v-if="selected === 'twin-json'"
          class="chip"
          :class="{ active: objectLabels }"
          :aria-pressed="objectLabels"
          @click="objectLabels = !objectLabels"
        >
          Objects
        </button>
      </div>
    </div>
  </section>
</template>

<style scoped>
.panel {
  display: flex;
  flex-direction: column;
  min-width: 0;
  min-height: 0;
  background: var(--panel);
}

header {
  display: flex;
  align-items: center;
  gap: 16px;
  height: 36px;
  padding: 0 12px;
  border-bottom: 1px solid var(--line);
}

.tabs {
  display: flex;
  gap: 4px;
  overflow-x: auto;
}

.tabs .chip {
  font-size: 11px;
}

.stage {
  position: relative;
  flex: 1;
  min-height: 0;
}

.empty {
  position: absolute;
  inset: 0;
  display: grid;
  place-items: center;
  margin: 0;
  color: var(--text-faint);
  pointer-events: none;
}

.scale {
  position: absolute;
  left: 12px;
  bottom: 14px;
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 10px;
}

.scale i {
  width: 32px;
  height: 2px;
  background: var(--accent);
}

.look {
  position: absolute;
  right: 10px;
  bottom: 10px;
  display: flex;
  gap: 4px;
  padding: 4px;
  border: 1px solid var(--line-strong);
  border-radius: 4px;
  background: rgb(15 22 29 / 0.9);
}

.look .chip {
  font-size: 11px;
}
</style>
