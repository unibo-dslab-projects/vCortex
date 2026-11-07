<script setup lang="ts">
import { onBeforeUnmount, onMounted, ref, useTemplateRef } from 'vue'
import SceneCanvas from '@/components/SceneCanvas.vue'
import ViewTools from '@/components/ViewTools.vue'
import { PointCloud } from '@/scene/pointCloud'
import { openRawSocket } from '@/services/rawSocket'
import { mapBoxToThree, type ViewMode } from '@/scene/view'

const cloud = new PointCloud()
const scene = useTemplateRef('scene')
const count = ref(0)
const seq = ref<number | null>(null)
const runId = ref<number | null>(null)
const open = ref(false)
const floorZ = ref(0)
const mode = ref<ViewMode>('iso')

const view = (next: ViewMode) => {
  mode.value = next
  scene.value?.frame(mapBoxToThree(cloud.min, cloud.max), next)
}

let close = () => {}

onMounted(() => {
  close = openRawSocket({
    full(points, run, s) {
      const firstMap = runId.value !== run
      cloud.reset(points)
      count.value = cloud.count
      seq.value = s
      runId.value = run
      floorZ.value = cloud.zLow
      // a reconnect resends the same map: keep the camera where the user left it
      if (firstMap) view(mode.value)
    },
    delta(records, indices, s) {
      cloud.upsert(records, indices)
      count.value = cloud.count
      seq.value = s
    },
    connection(state) {
      open.value = state
    },
  })
})

onBeforeUnmount(() => {
  close()
  cloud.dispose()
})
</script>

<template>
  <section class="panel">
    <header>
      <span class="panel-title"><span class="dot" :class="{ on: open }" />Raw cloud (live)</span>
      <span class="hint">
        {{ count.toLocaleString('en') }} pts<template v-if="seq !== null">
          · seq {{ seq }}</template
        >
      </span>
    </header>
    <div class="stage">
      <SceneCanvas ref="scene" :floor-z="floorZ">
        <primitive :object="cloud.object" />
      </SceneCanvas>
      <ViewTools :mode="mode" @view="view" />
      <div class="badge chip" :class="{ active: open }">
        <span class="dot" :class="{ accent: open }" />
        {{ open ? (count ? 'Receiving map updates' : 'Waiting for the map') : 'Reconnecting…' }}
      </div>
      <div class="legend">
        <span>floor</span>
        <i />
        <span>ceiling</span>
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
  justify-content: space-between;
  gap: 12px;
  height: 36px;
  padding: 0 12px;
  border-bottom: 1px solid var(--line);
}

.stage {
  position: relative;
  flex: 1;
  min-height: 0;
}

.badge {
  position: absolute;
  left: 10px;
  bottom: 10px;
}

.legend {
  position: absolute;
  right: 10px;
  bottom: 12px;
  display: flex;
  align-items: center;
  gap: 6px;
  color: var(--text-faint);
  font-size: 10px;
}

.legend i {
  width: 70px;
  height: 4px;
  border-radius: 2px;
  background: linear-gradient(90deg, #0b3a46, #22d3ee 60%, #e0fbff);
}
</style>
