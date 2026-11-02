<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref, shallowRef } from 'vue'
import AppHeader from '@/components/AppHeader.vue'
import RawView from '@/components/RawView.vue'
import StatusBar from '@/components/StatusBar.vue'
import TwinView from '@/components/TwinView.vue'
import {
  getFormats,
  getSettings,
  getStatus,
  type Format,
  type Settings,
  type Status,
} from '@/services/api'

const STATUS_POLL_MS = 2000

const status = shallowRef<Status | null>(null)
const reachable = ref(true)
const formats = ref<Format[]>([])
const settings = ref<Settings | null>(null)
const selected = ref('twin-json')
const format = computed(() => formats.value.find((f) => f.id === selected.value))

let poll: ReturnType<typeof setInterval> | undefined

async function refresh() {
  try {
    status.value = await getStatus()
    reachable.value = true
    // formats and settings come once the backend answers, also after it was down at load
    if (!formats.value.length) formats.value = await getFormats()
    if (!settings.value) settings.value = await getSettings()
  } catch {
    reachable.value = false
  }
}

onMounted(() => {
  refresh()
  poll = setInterval(refresh, STATUS_POLL_MS)
})
onBeforeUnmount(() => clearInterval(poll))
</script>

<template>
  <div class="app">
    <AppHeader :status="status" :reachable="reachable" :format="format" />
    <main class="views">
      <RawView />
      <TwinView v-model="selected" :formats="formats" />
    </main>
    <StatusBar v-model:settings="settings" :status="status" />
  </div>
</template>

<style scoped>
.app {
  display: grid;
  grid-template-rows: auto 1fr auto;
  height: 100dvh;
}

.views {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1px;
  min-height: 0;
  background: var(--line);
}

@media (max-width: 900px) {
  .views {
    grid-template-columns: 1fr;
    grid-template-rows: 1fr 1fr;
  }
}
</style>
